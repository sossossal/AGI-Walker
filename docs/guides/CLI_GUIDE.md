# AGI-Walker CLI 娴ｈ法鏁ら幐鍥у础

## 韫囶偊鈧喎绱戞慨?

### 鐎瑰顥?

CLI瀹搞儱鍙垮鎻掑敶缂冾喖婀狝GI-Walker娑?閺冪娀娓舵０婵嗩樆鐎瑰顥婇妴?

### 閸╃儤婀伴悽銊︾《

# CLI 閺€顖涘瘮娑撱倕顨滈崗銉ュ經閿涘瞼鐡戦弫鍫熷⒔鐞?```bash
# 闁氨鏁?Skills 閸涙垝鎶?
python -m agi_walker.cli skills <command>

# 閻╁瓨甯寸拫鍐暏 workflows 鐎涙劕鎳℃禒銈忕礄缁涘鐜禍?skills workflows閿?python -m agi_walker.cli workflows <subcommand>

# 閹存牔濞囬悽銊ユ彥閹圭柉鍓奸張?agi_walker.bat skills <command>
agi_walker.bat workflows <subcommand>
```

---

## 閸涙垝鎶ら崣鍌濃偓?
### 1. 閸掓鍤幍鈧張濉杒ills

```bash
python -m agi_walker.cli skills list
```

**鏉堟挸鍤粈杞扮伐:**
```
閸欘垳鏁?Skills (3 娑?:

閵嗘劕缂撳Ο掳鈧?
  棣冾樆 robot-modeling
    韫囶偊鈧喎鍨卞鍝勫蓟鐡?閸ユ稖鍐?鏉烆喖绱￠張鍝勬珤娴滅儤膩閸?..

閵嗘劒绱崠鏍モ偓?
  閳挎瑱绗?parameter-optimizer
    閼奉亜濮╂导妯哄閺堝搫娅掓禍鍝勫棘閺?鐠愩劑鍣洪崚鍡楃/PID婢х偟娉?...

閵嗘劘娴嗛幑顫偓?
  棣冩惈 urdf-generator
    鐏忓棝鍘ょ純顔挎祮閹诡澀璐烾RDF/SDF閺嶇厧绱?..
```

**闁銆?**
- `-v, --verbose` - 閺勫墽銇氱拠锔剧矎娣団剝浼?
- `--category <閸掑棛琚?` - 閹稿鍨庣猾鏄忕箖濠?

**缁€杞扮伐:**
```bash
# 鐠囷妇绮忛崚妤勩€?
python -m agi_walker.cli skills list -v

# 閸欘亞婀呭鐑樐佺猾绫筴ills
python -m agi_walker.cli skills list --category 瀵ょ儤膩
```

---

### 2. 閺屻儳婀匰kill鐠囷附鍎?

```bash
python -m agi_walker.cli skills info <skill閸氬秶袨>
```

**缁€杞扮伐:**
```bash
python -m agi_walker.cli skills info robot-modeling
```

**鏉堟挸鍤?**
```
棣冾樆 robot-modeling
============================================================
閸氬秶袨: robot-modeling
閸掑棛琚? 瀵ょ儤膩
閹诲繗鍫? 韫囶偊鈧喎鍨卞鍝勫蓟鐡?閸ユ稖鍐?..
鐠侯垰绶? agi_walker\skills\robot-modeling

娓氭繆绂?
  python_modules: numpy, pydantic

閸欘垳鏁ら懘姘拱:
  - (閺?

閸欏倽鈧啯鏋冨?
  - api.md
```

**闁銆?**
- `-d, --doc` - 閺勫墽銇氱€瑰本鏆KILL.md閺傚洦銆?

**缁€杞扮伐:**
```bash
# 閺屻儳婀呯€瑰本鏆ｉ弬鍥ㄣ€?
python -m agi_walker.cli skills info robot-modeling -d
```

---

### 3. 閹兼粎鍌⊿kills

```bash
python -m agi_walker.cli skills search <閸忔娊鏁拠?
```

**缁€杞扮伐:**
```bash
python -m agi_walker.cli skills search 娴兼ê瀵?
python -m agi_walker.cli skills search URDF
python -m agi_walker.cli skills search 閺堝搫娅掓禍?
```

**鏉堟挸鍤?**
```
閹兼粎鍌ㄧ紒鎾寸亯 (2 娑?:

閳挎瑱绗?parameter-optimizer
  閼奉亜濮╂导妯哄閺堝搫娅掓禍鍝勫棘閺?..

棣冩惈 urdf-generator
  鐏忓棝鍘ょ純顔挎祮閹诡澀璐烾RDF/SDF閺嶇厧绱?..
```

---

### 4. 閸掓鍤崚鍡欒

```bash
python -m agi_walker.cli skills categories
```

**鏉堟挸鍤?**
```
Skill 閸掑棛琚?(3 娑?:

  娴兼ê瀵?(1 娑撶尰kills)
  瀵ょ儤膩 (1 娑撶尰kills)
  鏉烆剚宕?(1 娑撶尰kills)
```

---

### 5. 妤犲矁鐦夐柊宥囩枂

濡偓閺屻儲澧嶉張濉籯ills閻ㄥ嫪绶风挧鏍ㄦЦ閸氾附寮х搾鐐解偓?

```bash
python -m agi_walker.cli skills validate
```

**鏉堟挸鍤?**
```
妤犲矁鐦?Skills 闁板秶鐤?..

閴?閹碘偓閺堝』kills闁板秶鐤嗛張澶嬫櫏
```

**闁銆?**
- `-v, --verbose` - 閺勫墽銇氶幍鈧張澶愮崣鐠囦胶绮ㄩ弸?

**缁€杞扮伐:**
```bash
python -m agi_walker.cli skills validate -v
```

---

### 6. Workflows 韫囶偊鈧喖鈧岸浜?

```bash
python -m agi_walker workflows list
python -m agi_walker workflows run simulation_ready_robot
python -m agi_walker workflows validate robot_creation_pipeline
```

鐠?alias 閻╁瓨甯撮弰鐘茬殸閸?`skills workflows`閿涘本澧嶆禒銉ょ閸撳秶娈?workflow 鐎涙劕鎳℃禒銈嗗閺堝鈧銆嶇悰灞艰礋娑撯偓濡€茬閺嶆灚鈧?
---

### 7. Workflow 閹笛嗩攽缁涙牜鏆?

`workflows run` 閻滄澘婀弨顖涘瘮閺勬儳绱￠幐鍥х暰閹笛嗩攽缁涙牜鏆愰敍宀勪缉閸忓秮鈧粌鍑￠張澶夐獓閻椻晜妞傞崚鏉跨俺閺勵垶鍣哥捄鎴ｇ箷閺勵垵鐑︽潻鍥ｂ偓婵婄箹缁夊秹娈ｅ蹇氼攽娑撴亽鈧?
#### 姒涙顓荤悰灞艰礋: `resume`

```bash
python -m agi_walker.cli skills workflows run robot_creation_pipeline
python -m agi_walker.cli workflows run robot_creation_pipeline --resume
```

- 闁洤鍩屽鑼病鐎涙ê婀稉鏃堟姜缁岃櫣娈?`output_file` 閺冭绱濆銉╊€冩导姘垼鐠侀璐?`SKIPPED`
- `SKIPPED` 閻滄澘婀粻妤佸灇閸旂喓绮撻幀渚婄礉閹碘偓娴犮儰绗夋导姘晙閸戣櫣骞?閳ユ竷ompleted + 0.0% success閳?鏉╂瑧顫掔拠顖氼嚤閹呯波閺?
#### 瀵搫鍩楅柌宥堢獓: `force`

```bash
python -m agi_walker.cli skills workflows run robot_creation_pipeline --force
```

- 韫囩晫鏆愬鍙夋箒娴溠呭⒖
- 閹碘偓閺堝顒炴銈夊厴娴兼岸鍣搁弬鐗堝⒔鐞?- 闁倸鎮庨崶鐐茬秺濞村鐦崪宀勫櫢閺傛壆鏁撻幋鎰獓閻?
#### 闂呮梻顬囨潏鎾冲毉閻╊喖缍?

婵″倹鐏夋担鐘辩瑝閹櫕钖勯弻鎾茬波鎼存捇绮拋銈囨畱 `.output/` 閸?`exports/`閿涘苯褰叉禒銉﹀Ω閻╃顕潏鎾冲毉鐠侯垰绶為柌宥呯暰閸氭垵鍩岄弬鎵畱閺嶅湱娲拌ぐ鏇窗

```bash
python -m agi_walker.cli skills workflows run robot_creation_pipeline ^
  --force ^
  --output-root test_env/workflow_runs/run_001
```

閺佸牊鐏夐敍?
- `.output/created_robot.json` 娴兼艾鍟撻崚?`test_env/workflow_runs/run_001/.output/created_robot.json`
- `exports/robot.urdf` 娴兼艾鍟撻崚?`test_env/workflow_runs/run_001/exports/robot.urdf`
- workflow log 娑旂喍绱版潻娑樺弳鐠囥儵娈х粋鑽ゆ窗瑜版洑绗呴惃?`.output/`

---

### 8. 閺堚偓鐏?Smoke 妤犲本鏁?

閹恒劏宕橀幎濠佺瑓闂堛垼绻栨稉顏囧壖閺堫兛缍旀稉鐑樻付鐏忓繐褰叉穱锟犵崣閺€璺哄弳閸欙綇绱?

```bash
python tests/run_smoke_tests.py
```

鏉╂瑤閲滈懘姘拱娴兼碍顥呴弻銉窗

- Skills CLI 閺勵垰鎯侀崣顖氬灙閸戝搫鍞寸€?- Skills 闁板秶鐤嗛弰顖氭儊閺堝鏅?
- mock workflow 閺勵垰鎯侀崣顖濈箥鐞?- real workflow 閺勵垰鎯侀崣顖濈箥鐞?- Web 闂堛垺婢樼€电厧鍙嗛崪?`WsMessage(type="ping")` 閸忕厧顔愰幀褎妲搁崥锔筋劀鐢?- Godot Agent fake backend 閻ㄥ嫭娓剁亸?Web/API 閸欘垳鏁ら幀?- 婵″倹鐏夌€涙ê婀?external `godot-agent` 閻╊喖缍嶉敍灞藉晙鏉╄棄濮?modern backend 閻ㄥ嫭膩閺?鐠佲€冲灊/閼奉亝顥?smoke

閼奉亜鐣炬稊?smoke 娴溠呭⒖閻╊喖缍嶉敍?
```bash
python tests/run_smoke_tests.py --output-root test_env/smoke_runs/manual
```

婵″倹鐏夌憰浣规▔瀵繑瀵氱€?modern `godot-agent` 閻╊喖缍嶉敍?
```bash
AGI_WALKER_SMOKE_GODOT_AGENT_DIR=/path/to/godot-agent python tests/run_smoke_tests.py
```


## 娴ｈ法鏁ら崷鐑樻珯

### 閸︾儤娅?: 韫囶偊鈧喐鐓￠幍缍璳ill

```bash
# 1. 閹兼粎鍌ㄩ惄绋垮彠skill
python -m agi_walker.cli skills search 瀵ょ儤膩

# 2. 閺屻儳婀呯拠锔藉剰
python -m agi_walker.cli skills info robot-modeling

# 3. 閺屻儳婀呯€瑰本鏆ｉ弬鍥ㄣ€?
python -m agi_walker.cli skills info robot-modeling -d
```

### 閸︾儤娅?: 濡偓閺屻儳閮寸紒鐔哄Ц閹?
```bash
# 閸掓鍤幍鈧張濉籯ills
python -m agi_walker.cli skills list

# 妤犲矁鐦夐柊宥囩枂
python -m agi_walker.cli skills validate -v

# 鏉╂劘顢戦張鈧亸?smoke 妤犲本鏁?
python tests/run_smoke_tests.py
```

### 閸︾儤娅?: 濞村繗顫嶉弬鍥ㄣ€?

```bash
# 閸掓鍤鐑樐佺猾绫筴ills
python -m agi_walker.cli skills list --category 瀵ょ儤膩

# 閺屻儳婀呭В蹇庨嚋skill閻ㄥ嫯顕涢幆?
python -m agi_walker.cli skills info robot-modeling
python -m agi_walker.cli skills info parameter-optimizer
python -m agi_walker.cli skills info urdf-generator
```

---

## 韫囶偅宓庨懘姘拱

### Windows

娴ｈ法鏁?`agi_walker.bat`:

```bash
agi_walker.bat skills list
agi_walker.bat skills info robot-modeling
agi_walker.bat skills search 娴兼ê瀵?
```

### Linux/macOS (TODO)

閸掓稑缂?`agi_walker.sh`:

```bash
#!/bin/bash
python -m agi_walker.cli "$@"
```

娴ｈ法鏁?
```bash
chmod +x agi_walker.sh
./agi_walker.sh skills list
```

---

## 閹绘劗銇氶崪灞惧Η瀹?

### 1. 韫囶偊鈧喐鐓￠惇瀣簻閸?

```bash
python -m agi_walker.cli --help
python -m agi_walker.cli skills --help
python -m agi_walker.cli skills list --help
```

### 2. 鏉堟挸鍤柌宥呯暰閸?

```bash
# 娣囨繂鐡╯kills閸掓銆?
python -m agi_walker.cli skills list > skills_list.txt

# 娣囨繂鐡╯kill閺傚洦銆?
python -m agi_walker.cli skills info robot-modeling -d > robot_modeling_doc.md
```

### 3. 缁狅繝浜鹃幙宥勭稊

```bash
# 閹兼粎鍌ㄩ獮鍓佺埠鐠?
python -m agi_walker.cli skills search 閺堝搫娅掓禍?| find /c "skill"

# 鏉╁洦鎶ょ紒鎾寸亯
python -m agi_walker.cli skills list -v | findstr "瀵ょ儤膩"
```

---

## 鐢瓕顫嗛梻顕€顣?

**Q: 婵″倷缍嶉弻銉ф箙skill閻ㄥ嫯鍓奸張?**
A: 娴ｈ法鏁?`info` 閸涙垝鎶ゆ导姘灙閸戝搫褰查悽銊ㄥ壖閺?閻掕泛鎮楅崚?`agi_walker/skills/<skill-name>/scripts/` 閻╊喖缍嶉弻銉ф箙閵?

**Q: 婵″倷缍嶆潻鎰攽skill閻ㄥ嫯鍓奸張?**
A: 閻╁瓨甯存担璺ㄦ暏Python鏉╂劘顢?娓氬顩?
```bash
python agi_walker/skills/parameter-optimizer/scripts/batch_optimize.py --help
```

**Q: 婵″倷缍嶅ǎ璇插閺傛壆娈憇kill?**
A: 閸欏倽鈧?`.agent/AGENTS.md` 娑擃厾娈慡kills瀵偓閸欐垶瀵氶崡妞尖偓?

---

## 娑撳绔村?

- **GUI瀹搞儱鍙?*: 娴ｈ法鏁ら崶鎯ц埌閻ｅ矂娼板ù蹇氼潔閸滃奔濞囬悽鈯縦ills
- **闂嗗棙鍨氶崚鐧怐E**: 閸︺劋鍞惍浣虹椽鏉堟垵娅掓稉顓犳纯閹恒儴顔栭梻鐢媖ills
- **CI/CD闂嗗棙鍨?*: 閸︺劏鍤滈崝銊ュ濞翠胶鈻兼稉顓濆▏閻⑩€揕I瀹搞儱鍙?

---

**閻╃鍙ч弬鍥ㄣ€?**
- `.agent/AGENTS.md` - Skills缁崵绮虹€瑰本鏆ｉ幐鍥у础
- `agi_walker/skills/*/SKILL.md` - 閸氬嚪kill閻ㄥ嫪濞囬悽銊︽瀮濡?
- `examples/*.py` - Python娴狅絿鐖滅粈杞扮伐
