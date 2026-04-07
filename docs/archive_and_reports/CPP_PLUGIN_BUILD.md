# C++ 閹绘帊娆㈢紓鏍槯韫囶偊鈧喐瀵氶崡?

閺堫剚瀵氶崡妤€搴滈崝鈺傚亶缂傛牞鐦?AGI-Walker 閻?GDExtension C++ 閹绘帊娆㈤敍灞惧絹閸楀洨澧块悶鍡樐侀幏鐔糕偓褑鍏樼痪?**10閸?*閵?

---

## 棣冩惖 閸撳秶鐤嗙憰浣圭湴

### Windows

```powershell
# 鐎瑰顥?MinGW-w64閿涘牊甯归懡鎰▏閻?MSYS2閿?
# 鐠佸潡妫? https://www.msys2.org/

# 閸?MSYS2 娑擃厼鐣ㄧ憗鍛紣閸?
pacman -S mingw-w64-x86_64-gcc
pacman -S mingw-w64-x86_64-cmake
pacman -S mingw-w64-x86_64-python-scons

# 濞ｈ濮為崚?PATH
# C:\msys64\mingw64\bin
```

### Linux

```bash
sudo apt-get update
sudo apt-get install build-essential scons pkg-config libx11-dev libxcursor-dev \
    libxinerama-dev libgl1-mesa-dev libglu-dev libasound2-dev libpulse-dev \
    libudev-dev libxi-dev libxrandr-dev
```

### macOS

```bash
brew install scons
xcode-select --install
```

---

## 棣冩暏 缂傛牞鐦у銉╊€?

### 濮濄儵顎?1: 閸掓繂顫愰崠鏍х摍濡€虫健

```bash
cd d:\閺傛澘缂撻弬鍥︽婢剁AGI-Walker

# 婵″倹鐏夋潻妯绘弓閸忓娈曠€涙劖膩閸?
git submodule update --init --recursive
```

### 濮濄儵顎?2: 缂傛牞鐦?godot-cpp

```bash
cd gdextension_src/godot-cpp

# Windows (MinGW)
scons platform=windows target=template_debug
scons platform=windows target=template_release

# Linux
scons platform=linux target=template_debug
scons platform=linux target=template_release

# macOS
scons platform=macos target=template_debug
scons platform=macos target=template_release
```

**缂傛牞鐦ч弮鍫曟？**: 缁?10-20 閸掑棝鎸撻敍鍫ヮ浕濞嗏槄绱?

### 濮濄儵顎?3: 缂傛牞鐦фい鍦窗閹绘帊娆?

```bash
cd ../  # 閸ョ偛鍩?gdextension_src

# Windows
scons platform=windows

# Linux
scons platform=linux

# macOS
scons platform=macos
```

**鏉堟挸鍤弬鍥︽**:
- Windows: `bin/libgdexample.windows.template_debug.x86_64.dll`
- Linux: `bin/libgdexample.linux.template_debug.x86_64.so`
- macOS: `bin/libgdexample.macos.template_debug.universal.dylib`

---

## 閴?妤犲矁鐦夌€瑰顥?

### 閸?Godot 娑擃厽绁寸拠?

1. **閹垫挸绱?Godot 妞ゅ湱娲?*
   ```bash
   # 娴ｈ法鏁?Godot 4.2+ 閹垫挸绱?
   godot --path d:\閺傛澘缂撻弬鍥︽婢剁AGI-Walker\godot_project
   ```

2. **濡偓閺屻儲褰冩禒?*
   - 妞ゅ湱娲扮拋鍓х枂 閳?閹绘帊娆?
   - 绾喛顓?"Robot Simulation Toolkit" 閺勫墽銇氶獮璺烘儙閻?

3. **鏉╂劘顢戝ù瀣槸閸︾儤娅?*
   - 閹垫挸绱?`scenes/test_physics.tscn`
   - 閹?F5 鏉╂劘顢?
   - 鐟欏倸鐧?FPS閿涘牆绨茬拠銉︽▔閽佹褰侀崡鍥风礆

---

## 棣冩偘 鐢瓕顫嗛梻顕€顣?

### 闂傤噣顣?1: `scons: command not found`

**鐟欙絽鍠?*:
```bash
# Windows
pip install scons

# Linux/macOS
sudo apt-get install scons  # Linux
brew install scons          # macOS
```

### 闂傤噣顣?2: 閹靛彞绗夐崚?Python

**鐟欙絽鍠?*:
```bash
# 绾喕绻?Python 3.6+ 瀹告彃鐣ㄧ憗?
python --version

# Windows: 濞ｈ濮?Python 閸?PATH
```

### 闂傤噣顣?3: 缂傛牞鐦ч柨娆掝嚖 - 閹靛彞绗夐崚鏉裤仈閺傚洣娆?

**鐟欙絽鍠?*:
```bash
# 绾喕绻氱€涙劖膩閸ф鍑″锝団€橀崚婵嗩潗閸?
cd gdextension_src/godot-cpp
git submodule update --init --recursive
```

### 闂傤噣顣?4: Godot 閺冪姵纭堕崝鐘烘祰閹绘帊娆?

**濡偓閺?*:
1. 閹绘帊娆㈤弬鍥︽閺勵垰鎯侀崷?`gdextension_src/bin/` 閻╊喖缍?
2. 閺傚洣娆㈤幍鈺佺潔閸氬秵妲搁崥锕€灏柊宥嗘惙娴ｆ粎閮寸紒?
3. Godot 閻楀牊婀伴弰顖氭儊 4.2+

---

## 棣冩惓 閹嗗厴鐎佃鐦?

缂傛牞鐦ч崜宥呮倵閹嗗厴鐎佃鐦敍鍫ヮ暕閺堢噦绱?

| 閹稿洦鐖?| Python/GDScript | C++ 閹绘帊娆?| 閹绘劕宕?|
|------|----------------|----------|------|
| 閻椻晝鎮婂銉ㄧ箻 | ~30 FPS | ~300 FPS | **10x** |
| 閻㈠灚婧€濡剝瀚?| ~50 FPS | ~500 FPS | **10x** |
| 閸愬懎鐡ㄩ崡鐘垫暏 | 500 MB | 200 MB | **2.5x** |

---

## 棣冨箚 娑撳绔村?

缂傛牞鐦ч幋鎰閸氬函绱?
1. 鏉╂劘顢戠€瑰本鏆ｇ拋顓犵矊閿涘牊鈧嗗厴閺囨潙鎻╅敍?
2. 濞村鐦径宥嗘絽閸︾儤娅欓敍鍫熸纯婢舵艾鑻熺悰宀€骞嗘晶鍐跨礆
3. 鐠嬪啯鏆ｉ悧鈺冩倞閸欏倹鏆熼敍鍫熸纯缁墽鈥橀敍?

---

## 棣冩憥 閸欏倽鈧啳绁弬?

- [GDExtension 鐎规ɑ鏌熼弬鍥ㄣ€俔(https://docs.godotengine.org/en/stable/tutorials/scripting/gdextension/index.html)
- [Godot-cpp GitHub](https://github.com/godotengine/godot-cpp)
- [鐠囷妇绮忛弸鍕紦閹稿洤宕(BUILD_GUIDE.md)

---

**闂団偓鐟曚礁搴滈崝鈺嬬吹**
- Discord: discord.gg/agi-walker
- GitHub Issues: github.com/agi-walker/agi-walker-sim/issues
