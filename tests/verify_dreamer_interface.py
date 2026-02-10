import time
import numpy as np
from algorithms.dreamer_env import DreamerEnv

def test_dreamer_interface():
    print("🧪 Verification: DreamerV3 Interface")
    
    # 1. Connect
    env = DreamerEnv(port=9000)
    print("   Attempting connection...")
    try:
        obs, _ = env.reset()
        print("   ✅ Connection Established & Reset Successful")
    except Exception as e:
        print(f"   ❌ Connection Failed: {e}")
        return

    # 2. Latency Test
    print("\n   📉 Running Latency Test (100 steps)...")
    latencies = []
    
    for i in range(100):
        # Random Action
        action = np.random.uniform(-1.0, 1.0, size=(12,))
        
        obs, reward, done, _, info = env.step(action)
        latencies.append(info['latency'])
        
        if i % 20 == 0:
            print(f"      Step {i}: Latency={info['latency']:.2f}ms, Reward={reward}")
            
    avg_latency = np.mean(latencies)
    print(f"\n   ✅ Average Latency: {avg_latency:.2f}ms")
    
    if avg_latency < 10.0:
        print("   🚀 Performance: EXCELLENT (<10ms)")
    elif avg_latency < 20.0:
        print("   ⚠️ Performance: GOOD (<20ms)")
    else:
        print("   ❌ Performance: POOR (>20ms)")
        
    env.close()

if __name__ == "__main__":
    test_dreamer_interface()
