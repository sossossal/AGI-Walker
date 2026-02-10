class OnboardingTour {
    constructor() {
        this.steps = [
            {
                target: null, // Center screen
                title: "👋 欢迎使用 AGI-Walker",
                content: "这是您的机器人控制中心。在这里，您可以设计、仿真和训练 AI 驱动的机器狗。",
                position: "center"
            },
            {
                target: "#godot-connect-btn",
                title: "🔌 连接仿真器",
                content: "首先，确保 Godot 仿真器已在运行，然后点击此按钮建立连接。连接后，您可以看到实时数据流。",
                position: "bottom"
            },
            {
                target: ".dashboard .card:nth-child(1)", // System Status Card
                title: "📊 监控状态",
                content: "这里显示系统的核心指标，如活跃训练任务、网络连接状态等。",
                position: "right"
            },
            {
                target: "#fetch-market-btn",
                title: "📦 获取零件",
                content: "点击刷新市场，从云端获取最新的电机、电池和传感器。导入后可用于设计新机器人。",
                position: "top"
            },
            {
                target: "button[onclick^='createTask']",
                title: "🚀 开始训练",
                content: "一切准备就绪后，点击这里创建一个新的强化学习训练任务。",
                position: "bottom"
            }
        ];

        this.currentStep = 0;
        this.overlay = null;
        this.box = null;
    }

    start() {
        if (localStorage.getItem('onboarding_completed') === 'true') {
            return;
        }
        this.showStep(0);
    }

    forceStart() {
        this.currentStep = 0;
        this.showStep(0);
    }

    showStep(index) {
        if (index >= this.steps.length) {
            this.finish();
            return;
        }

        this.currentStep = index;
        const step = this.steps[index];

        this._createOverlay();
        this._highlight(step.target);
        this._showBox(step);
    }

    finish() {
        localStorage.setItem('onboarding_completed', 'true');
        this._removeOverlay();
    }

    _createOverlay() {
        if (document.getElementById('onboarding-overlay')) return;

        const overlay = document.createElement('div');
        overlay.id = 'onboarding-overlay';
        overlay.className = 'onboarding-overlay';
        document.body.appendChild(overlay);
        this.overlay = overlay;
    }

    _removeOverlay() {
        if (this.overlay) this.overlay.remove();
        if (this.box) this.box.remove();

        // Remove highlighting
        document.querySelectorAll('.onboarding-highlight').forEach(el => {
            el.classList.remove('onboarding-highlight');
        });

        this.overlay = null;
        this.box = null;
    }

    _highlight(selector) {
        // Remove previous
        document.querySelectorAll('.onboarding-highlight').forEach(el => {
            el.classList.remove('onboarding-highlight');
        });

        if (selector) {
            const el = document.querySelector(selector);
            if (el) {
                el.classList.add('onboarding-highlight');
                el.scrollIntoView({ behavior: 'smooth', block: 'center' });
            }
        }
    }

    _showBox(step) {
        if (this.box) this.box.remove();

        const box = document.createElement('div');
        box.className = 'onboarding-box';

        let html = `
            <h3>${step.title}</h3>
            <p>${step.content}</p>
            <div class="onboarding-actions">
                <button onclick="tour.finish()" class="btn-skip">跳过</button>
                <div style="flex:1"></div>
        `;

        if (this.currentStep > 0) {
            html += `<button onclick="tour.showStep(${this.currentStep - 1})" class="btn-prev">上一步</button>`;
        }

        html += `<button onclick="tour.showStep(${this.currentStep + 1})" class="btn-next">
            ${this.currentStep === this.steps.length - 1 ? '完成' : '下一步'}
        </button>`;

        html += `</div>`;
        box.innerHTML = html;
        document.body.appendChild(box);
        this.box = box;

        // Positioning
        if (step.target) {
            const targetEl = document.querySelector(step.target);
            if (targetEl) {
                const rect = targetEl.getBoundingClientRect();
                // Simple positioning logic
                // Ideally use Popper.js, but manual for zero-dep
                let top = rect.bottom + 10;
                let left = rect.left;

                // Adjust if off screen
                if (left + 300 > window.innerWidth) left = window.innerWidth - 320;
                if (top + 200 > window.innerHeight) top = rect.top - 220;

                box.style.top = `${top + window.scrollY}px`;
                box.style.left = `${left + window.scrollX}px`;
                return;
            }
        }

        // Default center
        box.style.top = '50%';
        box.style.left = '50%';
        box.style.transform = 'translate(-50%, -50%)';
        box.style.position = 'fixed';
    }
}

const tour = new OnboardingTour();

// Auto start on load
window.addEventListener('DOMContentLoaded', () => {
    // Small delay to ensure rendering
    setTimeout(() => tour.start(), 1000);
});
