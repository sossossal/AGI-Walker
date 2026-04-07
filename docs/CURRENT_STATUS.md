# AGI-Walker V1 閻樿埖鈧礁顓虹懛鈧?

**閻楀牊婀?*: v1.0.0-RC  
**閺囧瓨鏌婇弮鍫曟？**: 2026-04-02

閺堫剚鏋冩禒鑸电垼韫囨娼?AGI-Walker V1 鐠佲€冲灊閻ㄥ嫭顒滃蹇氭彧閹存劑鈧倻绮℃潻?Week 1-8 閻ㄥ嫭鏁归崣锝呬紣娴ｆ粣绱濇い鍦窗瀹歌弓绮犻垾婊冪杽妤犲本鈧冨斧閸ㄥ鈧繃绱ㄦ潻娑楄礋閳ユ粌鍙挎径鍥ㄧ壋韫囧啴妫撮悳顖滄畱閺堝搫娅掓禍鍝勭紦濡€茬瑢娴犺濮熺化鑽ょ埠閳ユ縿鈧?
## V1 閺嶇绺炬潏鐐灇閹稿洦鐖?(100% 鐎瑰本鍨?

娴犮儰绗呮稉鑽ゅ殠閸旂喕鍏樺鏌モ偓姘崇箖閸忋劑鍣?Smoke Test 閸滃苯娲栬ぐ鎺楃崣鐠囦緤绱濈悮顐ヮ潒娑?V1 缁嬪啿鐣鹃崗銉ュ經閿?
- **Skills & Workflow 闂傤厾骞?*:
  - 閴?3 娑擃亝鐗宠箛?Skills (robot-modeling, parameter-optimizer, urdf-generator) 瀹稿弶鏁归崣锝冣偓?  - 閴?`robot_creation_pipeline` 鐎规ɑ鏌?Workflow 瀹歌尙菙鐎规熬绱濋弨顖涘瘮 real/mock 閸欏本膩閹笛嗩攽閵?  - 閴?CLI 閹稿洣鎶?(`skills list/validate`, `workflows run/status`) 閸忓嘲顦悽鐔堕獓缁狙囩灳濡帗鈧佲偓?
- **Web Workflow Console**:
  - 閴?`workflows.html` 瀹稿弶顒滃蹇庨獓閸濅礁瀵查敍灞炬暜閹镐礁鐤勯弮鑸垫）韫?(SSE)閵嗕浇绻嶇悰宀冾嚊閹懌鈧礁寮弫鎷岊洬閻╂牕鎷伴崣鏍ㄧХ/闁插秷鐦妴?  - 閴?娴溠呭⒖娣囨繄鏆€缁涙牜鏆?(Retention Strategy) 瀹告彃姘ㄧ紒顏庣礉姒涙顓绘穱婵堟殌 200 閺夆剝鍨?30 婢垛晞绻嶇悰宀冾唶瑜版洏鈧?
- **Godot 鐎规ɑ鏌熼柧鎹愮熅**:
  - 閴?`session_bridge` 瀹歌尙鈥樼粩瀣╄礋姒涙顓绘导鐘虹翻鐠侯垰绶為敍灞炬暜閹镐浇鍤滈崝銊ユ倱濮?Workflow 娴溠呭⒖閵?  - 閴?Headless Smoke 閻戠喖娴樺ù瀣槸瀹歌尪顩惄鏍у弿闁?lifecycle (launch/status/schema/load/step/stop)閵?  - 閴?`2026-04-02` 閸?Windows 閺堫剙婀撮悳顖氼暔娑撳鍣搁弬鏉跨杽濞村鈧俺绻?`tests/run_smoke_tests.py` 娑?`tests/test_godot_headless_smoke.py`閵?  - 閴?姒涙顓婚崷鐑樻珯 `demo_generated_biped.tscn` 娑?runner 閸︾儤娅?`run_rl_server.tscn` 閸у洤鍑￠柅姘崇箖 headless lifecycle 妤犲本鏁归妴?
- **Nightly 鏉╂劗娣梻顓犲箚**:
  - 閴?`Nightly 鏉╂劗娣い绀?(/static/nightly.html) 瀹歌弓绗傜痪鍖＄礉鐎圭偞妞傛潻鍊熼嚋 GitHub Actions 娑撴捇銆嶉崶鐐茬秺閵?  - 閴?閸忓嘲顦弽鍥у櫙閸栨牞鐦栭弬顓濋獓閻椻晙绗?Issue 濡剝婢橀敍灞炬暜閹镐礁鎻╅柅鐔奉槻閻滈绗岄弫鍛存閹烘帗鐓￠妴?
## 瑜版挸澧犳い鍦窗鐎规矮缍?

閹搭亣鍤?2026-04-02閿涘瓑GI-Walker v1.0.0 閺勵垽绱?

**娑撯偓娑擃亪娼伴崥鎴炴簚閸ｃ劋姹夊鐑樐侀妴浣稿棘閺侀绱崠鏍︾瑢 Godot 闂嗗棙鍨氭宀冪槈閻ㄥ嫭鐖ｉ崙鍡楀瀹搞儰缍斿ù浣搁挬閸欒埇鈧?*

鐎瑰啩璐熷鈧崣鎴ｂ偓鍛絹娓氭稐绨℃禒搴樷偓婊冨棘閺佹澘瀵茬拋鎹愵吀閳ユ繂鍩岄垾婊呭⒖閻炲棔璞㈤惇鐔肩崣鐠囦讲鈧繄娈戦張鈧亸蹇撳讲娣嚶ょ熅瀵板嫨鈧?
## 閺堚偓鏉╂垿鐛欓弨璺烘彥閻?
`2026-04-02` 閻ㄥ嫯藟閸忓懘鐛欓弨鍓佺波閺嬫粣绱?

- `python tests/run_smoke_tests.py --output-root test_env/smoke_runs/v1_plan_check_after_fix`閿涙岸鈧俺绻?
- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv`閿涙岸鈧俺绻?
- `AGI_WALKER_GODOT_HEADLESS_SCENE=run_rl_server.tscn` 閸氬骸顦茬捄鎴濇倱娑撯偓閺?headless smoke閿涙岸鈧俺绻?

鏉╂瑦鍓伴崨宕囨絻濮濄倕澧?V1 妤犲本鏁归崣锝呯窞娑擃厼鏁稉鈧棁鈧憰浣烘埛缂侇叀鐦夐弰搴ｆ畱 `godot-headless-smoke` 瀹告彃鐣幋鎰版４閻滎垶鐛欑拠浣碘偓?
## 2026-04-03 妫版繂顦诲Λ鈧?
- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1` 閸欑姴濮?`AGI_WALKER_GODOT_HEADLESS_SCENE=run_rl_server.tscn`閿涙eadless smoke 閸愬秵顐奸柅姘崇箖閿涘瞼鈥樼拋?runner 閸︾儤娅欑€瑰本鏆?lifecycle 閹存劕濮涢敍宀€鏁撻幋鎰畱 `headless_smoke_report.json` 娑?`failure_stage` 娑?`success`閵?- `docker compose -f docker-compose.prod.yml up --build`閿涙瓙eb API閵嗕簚orker閵嗕阜edis閵嗕赋ostgreSQL閵嗕赋rometheus 閸?Grafana 闂€婊冨剼闁€熷厴闁劎璁查獮鎯扮箻閸忋儱浠存惔椋庡Ц閹緤绱盙rafana 娴犲秵妫ゅ▔鏇犵拨 3000 缁旑垰褰涢崶鐘辫礋 Windows 閸?`netsh interface ipv4 show excludedportrange protocol=tcp` 娑擃厺绻氶悾?2970-3069閿涘本澧嶆禒銉ュ涧閼宠棄鐔€娴滃骸鍙炬禒鏍伂閸欙綇绱?001+閿涘鎯庨崝顭掔礉瀵板懐閮寸紒鐔侯吀閻炲棗鎲崇憴锝夋珟閹烘帡娅庨崥搴″晙閻劑绮拋銈囶伂閸欙綁鍣哥捄鎴欌偓?- `python tests/run_distributed_smoke.py --build`閿涙艾婀幒鍫滅埃 Docker 闁板秶鐤嗙拠璇插晸閺夊啴妾洪崥搴㈢€杞扮瑢 distributed smoke 閸у洭鈧俺绻冮敍瀹峝istributed_monitor.monitor_active`閵嗕笩odot actor閵嗕苟ightly status 闁姤妯夌粈?ready閿涘ummary 閺勫墽銇?`PASS`閵?
`2026-04-03` 閻ㄥ嫯绻栨禍娑欘梾妤犲矁绻樻稉鈧銉ョ彁閸ヨ桨绨?V1 閳ユ笡roduction smoke + godot headless + distributed path閳?閻ㄥ嫬褰查悽銊︹偓褋鈧?
## 閸樺棗褰堕弬鍥ㄣ€傜拠瀛樻

娴犳挸绨辨稉?`docs/archive_and_reports/` 娑?`docs/architecture/` 娑撳娈戦弮鈺傛埂閺傚洦銆?(2026-01 娴犮儱澧? 娴犲懍缍旀稉鐑樼川鏉╂稑寮懓鍐︹偓淇? 閻ㄥ嫭娼堟繛浣稿經瀵板嫯顕禒銉︽拱閺傚洣娆㈤崣濠冪壌閻╊喖缍?`README.md` 娑撳搫鍣妴?
## 閹恒劏宕橀梼鍛邦嚢妞ゅ搫绨?

1. [README.md](../README.md)
2. [CLI 娴ｈ法鏁ら幐鍥у础](guides/CLI_GUIDE.md)
3. [Web 闂堛垺婢橀幐鍥у础](guides/WEB_PANEL_GUIDE.md)
4. [闁劎璁查幍瀣斀](../PRODUCTION_DEPLOYMENT_RUNBOOK.md)
5. [Smoke 濞村鐦梿鍝?../tests/run_smoke_tests.py)
