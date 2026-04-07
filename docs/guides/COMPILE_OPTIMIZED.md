# C++ 閹绘帊娆㈡导妯哄缂傛牞鐦уù浣衡柤

## 閺嬪嫬缂撶化鑽ょ埠閸掑棙鐎?

**閸欐垹骞?*: 妞ゅ湱娲版担璺ㄦ暏 **CMake** 閺嬪嫬缂撶化鑽ょ埠閿涘牅绗夐弰?scons閿?

**娴兼ê濞?*:
- 閴?閺囨潙銈介惃?IDE 闂嗗棙鍨?
- 閴?鐠恒劌閽╅崣鐗堟暜閹?
- 閴?閼奉亜濮╂笟婵婄缁狅紕鎮?
- 閴?Visual Studio 閸樼喓鏁撻弨顖涘瘮

## 缂傛牞鐦у銉╊€冩导妯哄

### 闂冭埖顔?1: 閸戝棗顦敍鍫滅濞嗏剝鈧嶇礆
```powershell
# 1. 閸掓繂顫愰崠?godot-cpp 鐎涙劖膩閸?
cd d:\閺傛澘缂撻弬鍥︽婢剁AGI-Walker\gdextension_src
git submodule update --init --recursive

# 妫板嫯顓搁弮鍫曟？: 2-3 閸掑棝鎸撻敍鍫滅瑓鏉?godot-cpp閿?
```

### 闂冭埖顔?2: 闁板秶鐤嗛敍鍫滅濞嗏剝鈧嶇礆
```powershell
# 2. 閸掓稑缂撻弸鍕紦閻╊喖缍?
New-Item -ItemType Directory -Force -Path build
cd build

# 3. 閻㈢喐鍨?Visual Studio 妞ゅ湱娲?
cmake .. -G "Visual Studio 17 2022" -A x64

# 妫板嫯顓搁弮鍫曟？: 30 缁?
```

### 闂冭埖顔?3: 缂傛牞鐦?
```powershell
# 4a. Release 濡€崇础閿涘牊甯归懡鎰剁礉閻劋绨幀褑鍏樺ù瀣槸閿?
cmake --build . --config Release -- /m

# 4b. 閹?Debug 濡€崇础閿涘牆绱戦崣鎴ｇ殶鐠囨洜鏁ら敍?
cmake --build . --config Debug -- /m

# 妫板嫯顓搁弮鍫曟？:
# - 妫ｆ牗顐? 15-20 閸掑棝鎸撻敍鍫㈢椽鐠?godot-cpp閿?
# - 閸氬海鐢? 1-2 閸掑棝鎸撻敍鍫滅矌缂傛牞鐦ф穱顔芥暭闁劌鍨庨敍?
```

## 閹嗗厴娴兼ê瀵查崣鍌涙殶

### 楠炴儼顢戠紓鏍槯
```powershell
# /m 閺嶅洤绻?= 娴ｈ法鏁ら幍鈧張濉丳U閺嶇绺?
cmake --build . --config Release -- /m
```

### 缂傛牞鐦ч弮鍫曟？妫板嫪鍙?

| 闂冭埖顔?| 妫ｆ牗顐?| 婢х偤鍣?|
|------|------|------|
| godot-cpp | 10-15 閸掑棝鎸?| - |
| 妞ゅ湱娲版禒锝囩垳 | 1-2 閸掑棝鎸?| 10-30 缁?|
| **閹槒顓?* | **15-20 閸掑棝鎸?* | **< 1 閸掑棝鎸?* |

## 妤犲矁鐦夌紓鏍槯缂佹挻鐏?

```powershell
# 濡偓閺?DLL 閺傚洣娆?
Test-Path "..\godot_project\addons\robot_sim_toolkit\bin\robotparts.windows.x86_64.dll"

# 鎼存棁顕氭潻鏂挎礀: True
```

## 鐢瓕顫嗛柨娆掝嚖婢跺嫮鎮?

### 闁挎瑨顕?1: CMake not found
```powershell
# 鐎瑰顥?CMake
winget install Kitware.CMake
# 閹存牔绮?https://cmake.org/download/ 娑撳娴?
```

### 闁挎瑨顕?2: Visual Studio not found
```powershell
# 闁銆?1: 娴ｈ法鏁?MinGW閿涘牐顫?MINGW_BUILD_GUIDE.md閿?
# 闁銆?2: 鐎瑰顥?Visual Studio 2022 Community閿涘牆鍘ょ拹鐧哥礆
# https://visualstudio.microsoft.com/downloads/
```

### 闁挎瑨顕?3: godot-cpp not found
```powershell
# 闁插秵鏌婇崚婵嗩潗閸栨牕鐡欏Ο鈥虫健
git submodule update --init --recursive --force
```

## 娑撳绔村?

缂傛牞鐦ч幋鎰閸氬函绱?
1. 閸?Godot 缂傛牞绶崳銊よ厬閹垫挸绱戞い鍦窗
2. 閸氼垳鏁ら幓鎺嶆閿涘牓銆嶉惄顔款啎缂?閳?閹绘帊娆㈤敍?
3. 鏉╂劘顢戝ù瀣槸閸︾儤娅欐宀冪槈閹嗗厴閹绘劕宕?
