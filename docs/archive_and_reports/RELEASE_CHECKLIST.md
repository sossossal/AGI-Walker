# AGI-Walker Release Checklist

閸︺劋濞囬悽?`main` 閸掑棙鏁崣鎴濈閺傛壆澧楅張顒€澧犻敍宀冾嚞閸斺€崇箑绾喛顓绘禒銉ょ瑓娴滃銆嶉敍?

## 1. 娴狅絿鐖滅拹銊╁櫤闂傘劎顩?(CI Gates)
- [ ] **Lint Check**: `ruff` 閸?`black` 濡偓閺屻儱鍙忛柅姘崇箖 (閺?`continue-on-error`)閵?
- [ ] **Unit Tests**: 閹碘偓閺堝宕熼崗鍐╃ゴ鐠囨洖婀?Ubuntu 閸?Windows 娑撳﹤娼庢稉铏硅雹閼瑰眰鈧?
- [ ] **Coverage**: 閺嶇绺鹃崠?`agi_walker/` 鐟曞棛娲婇悳鍥彧閸掓澘鐔€缁?(瑜版挸澧犲楦款唴 > 60%)閵?

## 2. 閸旂喕鍏橀崶鐐茬秺妤犲矁鐦?
- [ ] **URDF Export**: 鏉╂劘顢?`verify_pipeline.py`閿涘瞼鈥樻穱婵堟晸閹存劗娈?URDF 缂佹挻鐎€瑰本鏆ｉ敍鍫濇儓 Thigh/Shin閿涘鈧?
- [ ] **Web Panel**: 閸氼垰濮?`server.py`閿涘瞼鈥樻穱?`design.html` 閼宠姤鍨氶崝鐔烘晸閹存劖婧€閸ｃ劋姹夐妴?
- [ ] **Examples**: 鏉╂劘顢?`python examples/skills_demo.py` 閺冪姵濮ら柨娆嶁偓?

## 3. 娓氭繆绂嗘稉搴ｅ箚婢?
- [ ] **Requirements**: `requirements.txt` 瀹告煡鏀ｇ€规矮绗栭弮鐘插暱缁愪降鈧?
- [ ] **PyTest**: 閺堫剙婀存潻鎰攽 `pytest` (娑撳秴鐢崣鍌涙殶) 鎼存棃绮拋銈勮礋 Pass閵?

## 4. 閺傚洦銆傛稉搴ｅ閺?
- [ ] **VERSION**: `VERSION` 閺傚洣娆㈤崘鍛啇瀹稿弶娲块弬鑸偓?
- [ ] **Changelog**: `CHANGELOG.md` 瀹歌尪顔囪ぐ鏇熸拱濞嗏€冲綁閺囨番鈧?
- [ ] **Docs**: 閺傚洦銆傛稉顓犳畱缁€杞扮伐娴狅絿鐖滃鎻掓倱濮濄儲娲块弬鑸偓?

---
> **濞夈劍鍓?*: 婵″倹鐏?CI 娑撹櫣瀛╅懝璇х礉**娑撱儳顩﹂崥鍫濊嫙**閸?`main` 閸掑棙鏁敍?
