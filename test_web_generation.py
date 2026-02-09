import requests
import json
import time

def test_generation():
    url = "http://localhost:8000/api/generate_robot"
    payload = {
        "name": "web_test_bot",
        "type": "quadruped",
        "scenario": "custom",
        "height": 0.35,
        "mass": 4.5,            # Custom override
        "material": "carbon_fiber" # Custom override
    }
    
    print(f"Sending request to {url}...")
    try:
        start_time = time.time()
        response = requests.post(url, json=payload)
        end_time = time.time()
        
        print(f"Status Code: {response.status_code}")
        print(f"Time Taken: {end_time - start_time:.2f}s")
        
        if response.status_code == 200:
            data = response.json()
            print("\nResponse Data:")
            print(json.dumps(data, indent=2, ensure_ascii=False))
            
            if data["status"] == "success":
                print("\n✅ Web Generation Test PASSED")
            else:
                print("\n❌ Web Generation Test FAILED (Logic Error)")
        else:
            print(f"\n❌ Web Generation Test FAILED (HTTP {response.status_code})")
            print(response.text)
            
    except Exception as e:
        print(f"\n❌ Connection Failed: {e}")
        print("Make sure server.py is running!")

if __name__ == "__main__":
    test_generation()
