/**
 * Web-Godot WebSocket 协议客户端 (JavaScript)
 * 
 * 实现 Web-Godot 协议 v1.0 的 Web 端 JavaScript 客户端，支持：
 * - 发送命令到 Godot 仿真器
 * - 接收遥测数据流和状态更新
 * - 自动请求-响应 ID 匹配
 * - 完整的错误处理和重连机制
 * 
 * 使用示例:
 *   const client = new WebGodotClient('session-1');
 *   client.connect('ws://localhost:8000');
 *   await client.startSimulation({ gravity: 9.81 });
 */

class WebGodotClient {
  constructor(sessionId = 'web_session_1') {
    this.sessionId = sessionId;
    this.ws = null;
    this.baseUrl = 'ws://localhost:8000';
    this.isConnected = false;
    this.messageId = 0;
    this.pendingRequests = new Map();
    this.eventCallbacks = {
      telemetry: [],
      schema: [],
      status: [],
      error: [],
      connected: [],
      disconnected: []
    };
    this.reconnectEnabled = true;
    this.reconnectDelay = 2000;
    this.maxReconnectAttempts = 5;
    this.currentReconnectAttempts = 0;
    this.reconnectTimer = null;
  }

  /**
   * 连接到 WebSocket 服务器
   */
  connect(url = null, onConnected = null, onDisconnected = null) {
    if (url) this.baseUrl = url;

    console.log(`[WebGodot] 正在连接到 ${this.baseUrl}/ws/${this.sessionId}`);

    return new Promise((resolve, reject) => {
      try {
        this.ws = new WebSocket(`${this.baseUrl}/ws/${this.sessionId}`);

        this.ws.onopen = () => {
          console.log('[WebGodot] 已连接到服务器');
          this.isConnected = true;
          this.currentReconnectAttempts = 0;
          
          this._triggerCallbacks('connected');
          if (onConnected) onConnected();
          
          resolve();
        };

        this.ws.onmessage = (event) => {
          this._handleMessage(event.data);
        };

        this.ws.onerror = (error) => {
          console.error('[WebGodot] WebSocket 错误:', error);
          this._triggerCallbacks('error', 'WebSocket error');
        };

        this.ws.onclose = () => {
          if (this.isConnected) {
            console.log('[WebGodot] 连接已关闭');
            this.isConnected = false;
            this._triggerCallbacks('disconnected');
            if (onDisconnected) onDisconnected();
            this._triggerReconnect();
          }
        };
      } catch (error) {
        console.error('[WebGodot] 连接异常:', error);
        reject(error);
      }
    });
  }

  /**
   * 断开连接
   */
  disconnect() {
    console.log('[WebGodot] 断开连接');
    this.reconnectEnabled = false;
    if (this.ws) {
      this.ws.close();
    }
    this.isConnected = false;
  }

  /**
   * 处理接收的消息
   */
  _handleMessage(dataStr) {
    try {
      const message = JSON.parse(dataStr);
      const { type, id, payload, status } = message;

      // 检查是否是响应消息
      if (id && this.pendingRequests.has(id)) {
        const { resolve, reject, timeout } = this.pendingRequests.get(id);
        this.pendingRequests.delete(id);
        
        clearTimeout(timeout);
        
        if (status === 'success') {
          resolve(message);
        } else {
          reject(new Error(payload?.error || 'Unknown error'));
        }
        return;
      }

      // 处理推送消息
      switch (type) {
        case 'schema.update':
          this._triggerCallbacks('schema', payload?.data);
          break;

        case 'telemetry.update':
          this._triggerCallbacks('telemetry', payload?.data);
          break;
        
        case 'simulation.status':
          this._triggerCallbacks('status', payload);
          break;
        
        case 'simulation.error':
          console.error('[WebGodot] 仿真错误:', payload?.error);
          this._triggerCallbacks('error', payload?.error);
          break;
        
        case 'connection.status':
          console.log('[WebGodot] 连接状态:', payload?.connected);
          break;
        
        case 'pong':
          console.debug('[WebGodot] Pong 响应');
          break;
        
        default:
          console.warn('[WebGodot] 未知消息类型:', type);
      }
    } catch (error) {
      console.error('[WebGodot] 消息处理错误:', error);
    }
  }

  /**
   * 发送命令并等待响应
   */
  async _sendCommand(type, payload, timeout = 10000) {
    const messageId = String(this.messageId++);
    
    const message = {
      type,
      id: messageId,
      timestamp: new Date().toISOString(),
      payload
    };

    return new Promise((resolve, reject) => {
      // 添加超时处理
      const timeoutHandle = setTimeout(() => {
        this.pendingRequests.delete(messageId);
        reject(new Error(`Command '${type}' timed out after ${timeout}ms`));
      }, timeout);

      // 添加到待处理请求
      this.pendingRequests.set(messageId, {
        resolve,
        reject,
        timeout: timeoutHandle
      });

      // 发送消息
      this._sendRawMessage(message);
    });
  }

  /**
   * 发送推送消息（不期望响应）
   */
  _sendPushMessage(type, payload) {
    const message = {
      type,
      id: String(this.messageId++),
      timestamp: new Date().toISOString(),
      payload,
      status: 'push'
    };

    this._sendRawMessage(message);
  }

  /**
   * 发送原始 JSON 消息
   */
  _sendRawMessage(message) {
    if (!this.isConnected || this.ws.readyState !== WebSocket.OPEN) {
      console.warn('[WebGodot] 未连接，无法发送消息');
      return false;
    }

    try {
      this.ws.send(JSON.stringify(message));
      return true;
    } catch (error) {
      console.error('[WebGodot] 发送消息失败:', error);
      return false;
    }
  }

  // === 公共 API ===

  /**
   * 启动仿真
   */
  async startSimulation(physics = {}) {
    console.log('[WebGodot] 发送启动仿真命令');
    return this._sendCommand('simulation.start', { physics });
  }

  /**
   * 停止仿真
   */
  async stopSimulation() {
    console.log('[WebGodot] 发送停止仿真命令');
    return this._sendCommand('simulation.stop', {});
  }

  /**
   * 加载机器人配置
   */
  async loadRobotConfig(robotConfig) {
    console.log('[WebGodot] 发送加载机器人配置命令');
    return this._sendCommand('config.load_robot', { robot_config: robotConfig });
  }

  /**
   * 更新参数
   */
  async updateParams(params) {
    console.log('[WebGodot] 发送参数更新命令');
    return this._sendCommand('params.update', { params });
  }

  /**
   * 发送 Ping 请求
   */
  async ping() {
    return this._sendCommand('ping', {});
  }

  // === 遥测和推送 ===

  /**
   * 注册遥测数据回调
   */
  onTelemetry(callback) {
    if (typeof callback === 'function') {
      this.eventCallbacks.telemetry.push(callback);
    }
    return this;
  }

  onSchema(callback) {
    if (typeof callback === 'function') {
      this.eventCallbacks.schema.push(callback);
    }
    return this;
  }

  /**
   * 注册状态更新回调
   */
  onStatus(callback) {
    if (typeof callback === 'function') {
      this.eventCallbacks.status.push(callback);
    }
    return this;
  }

  /**
   * 注册错误回调
   */
  onError(callback) {
    if (typeof callback === 'function') {
      this.eventCallbacks.error.push(callback);
    }
    return this;
  }

  /**
   * 注册连接回调
   */
  onConnected(callback) {
    if (typeof callback === 'function') {
      this.eventCallbacks.connected.push(callback);
    }
    return this;
  }

  /**
   * 注册断开连接回调
   */
  onDisconnected(callback) {
    if (typeof callback === 'function') {
      this.eventCallbacks.disconnected.push(callback);
    }
    return this;
  }

  /**
   * 触发事件回调
   */
  _triggerCallbacks(eventType, data = null) {
    const callbacks = this.eventCallbacks[eventType] || [];
    callbacks.forEach(callback => {
      try {
        callback(data);
      } catch (error) {
        console.error(`[WebGodot] 事件回调错误 (${eventType}):`, error);
      }
    });
  }

  // === 重连机制 ===

  /**
   * 触发重连
   */
  _triggerReconnect() {
    if (!this.reconnectEnabled || this.currentReconnectAttempts >= this.maxReconnectAttempts) {
      console.log('[WebGodot] 已达最大重连次数，停止重连');
      return;
    }

    this.currentReconnectAttempts++;
    console.log(
      `[WebGodot] 将在 ${this.reconnectDelay / 1000} 秒后重新连接 (尝试 ${this.currentReconnectAttempts}/${this.maxReconnectAttempts})`
    );

    this.reconnectTimer = setTimeout(() => {
      console.log('[WebGodot] 尝试重新连接...');
      this.connect();
    }, this.reconnectDelay);
  }

  /**
   * 清除重连计时器
   */
  clearReconnectTimer() {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  // === 状态查询 ===

  /**
   * 获取连接状态
   */
  getConnectionStatus() {
    return {
      connected: this.isConnected,
      url: `${this.baseUrl}/ws/${this.sessionId}`,
      sessionId: this.sessionId,
      reconnectAttempts: this.currentReconnectAttempts,
      pendingRequests: this.pendingRequests.size,
      messageCount: this.messageId
    };
  }

  /**
   * 设置基础 URL
   */
  setBaseUrl(url) {
    this.baseUrl = url;
    return this;
  }

  /**
   * 设置会话 ID
   */
  setSessionId(sessionId) {
    this.sessionId = sessionId;
    return this;
  }

  /**
   * 设置重连配置
   */
  setReconnectConfig(enabled = true, delay = 2000, maxAttempts = 5) {
    this.reconnectEnabled = enabled;
    this.reconnectDelay = delay;
    this.maxReconnectAttempts = maxAttempts;
    return this;
  }
}

// 导出为全局变量和 ES6 模块
if (typeof module !== 'undefined' && module.exports) {
  module.exports = WebGodotClient;
}

// 浏览器全局对象
if (typeof window !== 'undefined') {
  window.WebGodotClient = WebGodotClient;
}
