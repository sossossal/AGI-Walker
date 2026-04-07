# Web 閹貉冨煑闂堛垺婢橀幐鍥у础

AGI-Walker 閹绘劒绶垫禍鍡曠娑擃亜鐔€娴?Web 閻ㄥ嫭甯堕崚鍫曟桨閺夊尅绱濋悽銊ょ艾缁狅紕鎮婃禒璇插閵嗕焦鐓￠惇瀣€夐棃銏ｇカ濠ф劧绱濋獮鎯扮殶鐠?Web-Godot 闂嗗棙鍨氶惄绋垮彠閸旂喕鍏橀妴?
## 閸氼垰濮╅棃銏℃緲

閸︺劑銆嶉惄顔界壌閻╊喖缍嶆稉瀣箥鐞涘矉绱?
```bash
python web_panel/server.py
```
娑旂喎褰叉禒銉ゅ▏閻劍膩閸ф鏌熷蹇ョ窗

```bash
python -m web_panel.server
```

閸氼垰濮╅崥搴ゎ問闂?[http://localhost:8000](http://localhost:8000)閵?
鐠囧瓨妲戦敍?
- `web_panel/` 閸愬懎鍑＄紒蹇撳瘶閸?FastAPI 閺堝秴濮熼妴渚€娼ら幀渚€銆夐棃銏犳嫲 WebSocket 閸楀繗顔呮径鍕倞娴狅絿鐖滈妴?- 閻╁瓨甯撮崥顖氬З閺堝秴濮熸稉宥囩搼娴滃骸鐣弫?Godot 閼辨柨濮╁鏌ョ崣鐠囦緤绱盙odot 娓氀傜矝闂団偓鐟曚線顤傛径鏍箚婢у啩绗岄懕鏃囩殶閵?- 閸︺劍妯夊蹇涘帳缂冾喖褰查悽?Godot 閻滎垰顣ㄩ崥搴礉`session_bridge` 鐎规ɑ鏌熼柧鎹愮熅瀹歌尙绮￠柅姘崇箖閻喎鐤?headless smoke 妤犲矁鐦夐妴?- 婵″倹鐏夐崷?Windows 缂佸牏顏稉顓海閸掓壆绱惍渚€妫舵０姗堢礉娴兼ê鍘涙担璺ㄦ暏 UTF-8 缂佸牏顏妴?- Web 閺堝秴濮熼惃鍕兇缂佺喓濮搁幀浣瑰复閸欙絿骞囬崷銊ょ窗妫版繂顦婚幎銉ユ啞 `distributed_monitor` 閼宠棄濮忛悩鑸碘偓渚婄礉閻劋绨拠瀛樻瑜版挸澧犳潻鎰攽閻滎垰顣ㄩ弰顖氭儊閸忓嘲顦?Zenoh 閸掑棗绔峰蹇曟磧閹貉嗗厴閸旀稏鈧?- 婵″倹鐏夐柊宥囩枂娴?GitHub Actions nightly 閻樿埖鈧焦娼靛┃鎰剁礉妫ｆ牠銆夌化鑽ょ埠閻樿埖鈧礁宕辨潻妯圭窗閺勫墽銇?nightly 娑撴捇銆嶉崶鐐茬秺閹芥顩﹂妴?
## 瑜版挸澧犳い鐢告桨閸忋儱褰?

- `http://localhost:8000/static/index.html`
  娑撶粯甯堕崚璺哄酱閿涘苯瀵橀崥顐℃崲閸斅扳偓涓無dot 閻╃鍙ч崗銉ュ經閸?workflow 閹貉冨煑閸欐媽鐑︽潪顑锯偓?- `http://localhost:8000/static/workflows.html`
  workflow 閹貉冨煑閸欏府绱濋棃銏犳倻 workflow 閹笛嗩攽閵嗕胶濮搁幀浣圭叀閻鎷伴崢鍡楀蕉濡偓缁鳖潿鈧?- `http://localhost:8000/static/nightly.html`
  nightly 鏉╂劗娣い纰夌礉闂堛垹鎮滄稉鎾汇€嶉崶鐐茬秺閵嗕工rtifact 娑撳孩婀伴崷鏉款槻閻滄澘鍙嗛崣锝冣偓?
## 瑜版挸澧犻崣顖溾€樼拋銈囨畱閸愬懎顔?

- Web 閺堝秴濮熼崗銉ュ經鐎涙ê婀敍姝歸eb_panel/server.py`
- 閸楀繗顔呮径鍕倞濡€虫健鐎涙ê婀敍姝歸eb_panel/ws_protocol.py`
- 闂堟瑦鈧線銆夐棃銏ｇカ濠ф劕鐡ㄩ崷顭掔窗`web_panel/static/`
- 闁劌鍨庢い鐢告桨閸滃本甯撮崣锝夋桨閸?Web-Godot 閼辨棁鐨?
- workflow 閹貉冨煑閸欑増鏁幐浣告倵閸?workflow run閵嗕讣SE 閻樿埖鈧焦绁﹂妴浣哥杽閺冭埖妫╄箛妞尖偓浣规櫊闂呮粏鐦栭弬顓炴嫲閸樺棗褰?runs 濡偓缁?- workflow 閹貉冨煑閸欐壆骞囬崷銊︽暜閹镐焦濡搁張鍝勬珤娴滄椽鍘ょ純顔婚獓閻椻晠鈧俺绻冨锝呯础閸氬海顏捄顖滄暠闁礁绶?Godot
- `session_bridge` 閻滄澘婀弰?workflow -> Godot 鐎规ɑ鏌熼幒銊ㄥ礃鐠侯垰绶為敍瀹璭gacy controller 娑撴槒顩︽穱婵堟殌閸忕厧顔愰悽銊┾偓?- Godot Agent 闂嗗棙鍨氱仦鍌滃箛閸︺劍鏁幐?backend 瀹搞儱宸堕崚鍥ㄥ床閿涘苯褰查崷?legacy `godot_studio_agent` 娑撳骸顦婚柈?`godot-agent` 閸氬海顏稊瀣？閸掑洦宕?

## Workflow 閹貉冨煑閸?
`/static/workflows.html` 瑜版挸澧犲鑼病閸忓嘲顦稉鈧弶锛勬祲鐎电懓鐣弫瀵告畱 Web workflow 闂傤厾骞嗛敍?
- 閸掓鍤崣顖滄暏 workflows
- 娴?`real/mock` 濡€崇础鐟欙箑褰傞崥搴″酱 workflow
- 閺勬儳绱￠柅澶嬪閹笛嗩攽缁涙牜鏆愰敍姝歠orce` / `resume`
- 娴ｈ法鏁?`output_root` 闂呮梻顬囨禍褏澧块惄顔肩秿
- 閺屻儳婀呭銉ч獓鏉╂稑瀹抽妴浣哥杽閺冭埖妫╄箛妞尖偓浣规櫊闂呮粏鐦栭弬顓炴嫲閺堚偓缂?JSON 缂佹挻鐏?
- 娑撳娴?artifacts閵嗕簚orkflow log 閸滃苯鐣弫鏉戠杽閺冭埖妫╄箛?- 鐏?`robot_config` 缁?artifacts 闁礁绶?`legacy_controller` 閹?`session_bridge`
- 娴ｈ法鏁?`/api/workflows/runs/{run_id}/godot-sync` 閼奉亜濮╅柅澶嬪閹恒劏宕橀張鍝勬珤娴滄椽鍘ょ純顔婚獓閻椻晛鑻熼柅浣哥窔 Godot
- 鐠佹澘缍嶉張鈧潻鎴滅濞?Godot 娴溿倓绮紒鎾寸亯閿涘苯鑻熼崷?run 鐠囷附鍎忛柌灞界潔缁€?transport/session/schema 閻樿埖鈧?- run 鐠囷附鍎忛悳鏉挎躬鏉╂ü绱伴弰鍓с仛 Godot session 鐎圭偞妞傞悩鑸碘偓浣碘偓浣搞亼鐠愩儵妯佸▓鐐光偓渚€鍣哥拠鏇熷絹缁€鍝勬嫲闁插秷鐦崗銉ュ經
- 閸?workflow 鐎瑰本鍨氶崥搴″讲閻㈠崬澧犵粩顖濆殰閸斻劏袝閸?閳ユ粓鈧礁绶?Godot閳?- 闁俺绻?`scope/status/mode/date/text/only_failures` 濡偓缁便垹缍嬮崜?runs 娑撳骸缍婂?runs
- 鐎?runs 閸掓銆冮崚鍡涖€夊ù蹇氼潔

鐠囧瓨妲戦敍?
- 瑜版挸澧?workflow run 娴犲秶鏁?Web 閺堝秴濮熸潻娑氣柤缁狅紕鎮婇敍宀勨偓鍌氭値鐠嬪啳鐦妴浣圭川缁€鍝勬嫲閺堚偓鐏忓繑鎼锋担婊堟４閻滎垬鈧?- 閸︺劍妯夊蹇涘帳缂?Godot 閻滎垰顣ㄩ崥搴礉workflow -> `session_bridge` -> Godot 鏉╂瑦娼€规ɑ鏌熸稉鑽ゅ殠瀹歌尙绮℃潏鎯у煂 V1 妤犲本鏁规０鍕埂閵?- 鏉╂瑤绮涙稉宥囩搼娴滃簶鈧粏娉曢獮鍐插酱閵嗕線娴傞柊宥囩枂閵嗕椒鎹㈤幇蹇擃樆闁?Godot 閻滎垰顣ㄩ垾婵嬪厴瀹歌尙绮℃潏鎯у煂缂佺喍绔寸粙鍐茬暰娴溠冩惂閻樿埖鈧降鈧?
### Runs 閸掓銆冮崚鍡涖€?

`GET /api/workflows/runs` 閻滄澘婀弨顖涘瘮閿?
- `page`
- `page_size`
- 閸忕厧顔愰弮褍寮弫?`limit`閿涘奔绲剧€瑰啰骞囬崷銊х搼娴犺渹绨?`page_size`

鏉╂柨娲栫紒鎾寸亯娴兼艾瀵橀崥顐窗

- `total_count`
- `page`
- `page_size`
- `total_pages`
- `has_previous_page`
- `has_next_page`

### 瑜版帗銆傛穱婵堟殌缁涙牜鏆?

Web workflow run 鐠佹澘缍嶆妯款吇瑜版帗銆傞崚甯窗

```text
.output/web_workflow_archive/
```

姒涙顓绘穱婵堟殌缁涙牜鏆愰敍?
- 閺堚偓婢舵矮绻氶悾?`200` 閺夆€崇秺濡?runs
- 閺堚偓婢舵矮绻氶悾?`30` 婢?
閸欘垶鈧俺绻冮悳顖氼暔閸欐﹢鍣虹拫鍐╂殻閿?
- `AGI_WALKER_WEB_RUNS_PAGE_SIZE`閿涙岸绮拋銈嗙槨妞ゅ灚娼弫甯礉姒涙顓?`20`
- `AGI_WALKER_WEB_RUNS_MAX_PAGE_SIZE`閿涙艾鍘戠拋鍝ユ畱閺堚偓婢堆勭槨妞ゅ灚娼弫甯礉姒涙顓?`100`
- `AGI_WALKER_WEB_ARCHIVE_MAX_RUNS`閿涙艾缍婂锝嗘付婢舵矮绻氶悾娆忣樋鐏忔垶娼?run閿涘矂绮拋?`200`
- `AGI_WALKER_WEB_ARCHIVE_MAX_AGE_DAYS`閿涙艾缍婂锝嗘付婢舵矮绻氶悾娆忣樋鐏忔垵銇夐敍宀勭帛鐠?`30`

婵″倹鐏夋担鐘绘付鐟曚礁鍣径鍥劥缂冭尙骞嗘晶鍐ㄥ綁闁插繑鏋冩禒璁圭礉閸欘垳娲块幒銉ュ棘閼板喛绱?

- [`deployment/web_panel.env.example`](../../deployment/web_panel.env.example)

鏉╂劘顢戦弮鑸电叀閹甸箖銆庢惔蹇ョ窗

1. `AGI_WALKER_WEB_ENV_FILE`
2. `deployment/web_panel.env`
3. `deployment/web_panel.env.example`

鏉╂瑦鍓伴崨宕囨絻閿?
- 閻╁瓨甯存潻鎰攽 `python -m web_panel.server` 閺冭绱漺orkflow API 娴兼俺鍤滈崝銊ョ毦鐠囨洝顕伴崣鏍箹娴滄稒鏋冩禒?- `quick_start.sh` / `quick_start.bat` 閻滄澘婀稊鐔剁窗閸︺劌鎯庨崝銊ュ閺勬儳绱＄拋鍓х枂 `AGI_WALKER_WEB_ENV_FILE`

### Docker 鏉╂劘顢戦崣妯圭秼

闁劎璁查惄顔肩秿閻滄澘婀幓鎰返娑撱倖娼?Web 闂堛垺婢樼€圭懓娅掔捄顖氱窞閿?
- 姒涙顓?`web-panel`
  娴ｈ法鏁ら弽绋跨妇娓氭繆绂嗛梿鍡礉娴兼ê鍘涙穱婵婄槈 workflow 閹貉冨煑閸欐澘鎷伴崺铏诡攨妞ょ敻娼扮粙鍐茬暰閸氼垰濮╅妴?- 閸欘垶鈧?`web-panel-distributed`
  闁俺绻?compose `distributed` profile 閸氼垰濮╅敍宀勵杺婢舵牕鐣ㄧ憗?`eclipse-zenoh`閿涘瞼鏁ゆ禍搴ㄦ付鐟曚礁顔愰崳銊ュ敶 Zenoh 閸掑棗绔峰蹇曟磧閹貉呮畱閸︾儤娅欓妴?
鐢摜鏁ら崨鎴掓姢閿?
```bash
docker compose -f deployment/docker-compose.yml up -d web-panel
docker compose -f deployment/docker-compose.yml --profile distributed up -d web-panel-distributed
python tests/run_distributed_smoke.py --build
```

姒涙顓荤拋鍧楁６閸︽澘娼冮敍?
- `web-panel`: `http://localhost:8080/static/index.html`
- `web-panel-distributed`: `http://localhost:8081/static/index.html`

鐠囧瓨妲戦敍?
- `python tests/run_distributed_smoke.py --build` 娴兼艾鎯庨悽?`distributed` 閸?`smoke` profiles閿涘苯鑻熼梽鍕敨閹峰鎹ｅù瀣槸娑撴挾鏁ら惃?`mock-godot`閵?- 鐠囥儴鍓奸張顒傛暏娴滃酣鐛欑拠?`sidecar-1 -> zenoh-router -> learner -> web-panel-distributed` 閺勵垰鎯侀惇鐔割劀閹垫捇鈧哎鈧?- 妤犲矁鐦夐柅姘崇箖閸氬函绱濋崣顖氭躬 `http://localhost:8081/api/distributed/status` 閻鍩屽ú鏄忕┈ actor閵?- 閼存碍婀伴崗銉ュ經娴ｅ秳绨?[`tests/run_distributed_smoke.py`](../../tests/run_distributed_smoke.py)閵?
閻╃鍙ч幒銉ュ經閿?
- `GET /api/system/status`
  鏉╂柨娲?`distributed_monitor` 鐎涙顔岄敍宀冾嚛閺?`zenoh_available`閵嗕梗monitor_active`閵嗕梗endpoint` 閸滃本娓舵潻鎴︽晩鐠囶垬鈧?- `GET /api/distributed/status`
  鏉╂柨娲?`actors` 娴犮儱寮?`monitor` 閻樿埖鈧焦鎲崇憰浣碘偓?
`/api/system/status` 閻滄澘婀潻妯圭窗妫版繂顦绘潻鏂挎礀 `nightly_regressions`閿涘瞼鏁ゆ禍搴ｇ舶妫ｆ牠銆夌化鑽ょ埠閻樿埖鈧礁宕遍弰鍓с仛閿?
- `Nightly 閸ョ偛缍奰
- `閺堚偓鏉╂垳绗撴い纭呯箥鐞涘畭
- `Smoke / Distributed / Godot`

閸氼垳鏁ら幍鈧棁鈧悳顖氼暔閸欐﹢鍣洪敍?
- `AGI_WALKER_GITHUB_REPO`
- `AGI_WALKER_GITHUB_TOKEN`閿涘苯褰查柅?- `AGI_WALKER_GITHUB_WORKFLOW_FILE`閿涘矂绮拋?`.github/workflows/ci.yml`
- `AGI_WALKER_NIGHTLY_STATUS_CACHE_SECONDS`閿涘矂绮拋?`300`

婵″倹鐏夋担鐘绘付鐟曚焦鐓￠惇瀣付鏉╂垵鍤戝▎鈥茬瑩妞?run閿涘矁鈧奔绗夋禒鍛Ц妫ｆ牠銆夐幗妯款洣閿涘苯褰叉担璺ㄦ暏閿?
- `GET /api/nightly/regressions?limit=6`

鏉╂瑤閲滈幒銉ュ經娴兼俺绻戦崶鐑囩窗

- 閺堚偓鏉╂垵鍤戝▎?`schedule / workflow_dispatch` 娑撴捇銆嶉崶鐐茬秺
- 濮ｅ繋閲?run 閻?`smoke / distributed-smoke / godot-headless-smoke` job 閻樿埖鈧?- 鐎电懓绨?artifact 閸氬秶袨
- 閹恒劏宕橀張顒€婀存径宥囧箛閸涙垝鎶?

閸掑棗绔峰蹇曟磧閹貉嗙箷娴兼艾顕梹鎸庢埂閺堫亝娲块弬鎵畱 actor 閸?TTL 濞撳懐鎮婇妴鍌炵帛鐠?TTL 娑?`30` 缁夋帪绱濋崣顖炩偓姘崇箖娴犮儰绗呴悳顖氼暔閸欐﹢鍣虹拫鍐╂殻閿?
- `AGI_WALKER_DISTRIBUTED_ACTOR_TTL_SECONDS`

## 瑜版挸澧犻幒銉ュ經缂佹挻鐎?

瑜版挸澧犳禒鎾崇氨闁插瞼娈?Godot 闂嗗棙鍨氶崚鍡曡礋娑撱倗顫掑Ο鈥崇础閿涘矁绻栨稉鈧悙褰掓付鐟曚焦妲戠涵顕嗙窗

### 1. Legacy Controller 濡€崇础

鏉╂瑦娼柧鎹愮熅闂堛垹鎮滈垾婊嗙箾閹?Godot閵嗕礁濮炴潪鑺ユ簚閸ｃ劋姹夐妴浣告儙閸?閸嬫粍顒涙禒璺ㄦ埂閵嗕焦娲块弬鏉垮棘閺佹壋鈧縿鈧?
鐎电懓绨查幒銉ュ經閿?- `POST /api/godot/connect`
- `POST /api/godot/disconnect`
- `GET /api/godot/status`
- `POST /api/godot/load-robot`
- `POST /api/godot/start`
- `POST /api/godot/stop`
- `POST /api/godot/update-params`

WebSocket 閸楀繗顔呮稉顓犳畱鏉╂瑤绨洪崨鎴掓姢娑旂喎鐫樻禍搴ょ箹娑撯偓缁紮绱?
- `simulation.start`
- `simulation.stop`
- `config.load_robot`
- `params.update`
- `ping`

### 2. Session Bridge 濡€崇础

鏉╂瑦娼柧鎹愮熅闂堛垹鎮滈垾婊勫瘻娴兼俺鐦介崥顖氬З Godot 鏉╂稓鈻奸妴渚€鈧俺绻?TCP 鐠囪褰囬柆銉︾ゴ/閸欐垿鈧礁濮╂担婧锯偓婵撶礉閺囧瓨甯存潻?RL 閹存牞鐨熺拠鏇熕夐敍灞芥倱閺冩湹绡冮弰顖氱秼閸?workflow -> Godot 閻ㄥ嫭甯归懡鎰暭閺傜鐭惧鍕┾偓?
鐎电懓绨查幒銉ュ經閿?- `POST /api/godot/{session_id}/launch`
- `POST /api/godot/{session_id}/stop`
- `GET /api/godot/{session_id}/status`
- `POST /api/godot/{session_id}/control`
- `WS /ws/{session_id}`

瑜版挸澧?`godot_project/scripts/tcp_server.gd` 閸欘垳鈥樼拋銈嗘暜閹镐胶娈?TCP 閸涙垝鎶ら弰顖ょ窗
- `reset`
- `step`
- `get_schema`
- `load_robot`

鏉╂瑦鍓伴崨宕囨絻 Session Bridge 瀹歌尙绮￠懗钘夘檮閹垫寧濯?workflow 娴溠呭⒖娑撳褰傛潻娆愭蒋娑撹崵鍤庨妴渚篹gacy Controller 娴犲秶鍔ф穱婵堟殌閿涘奔绲鹃弴鎾偓鍌氭値娴ｆ粈璐熼崗鐓庮啇閹恒儱褰涢敍宀冣偓灞肩瑝閺勵垰鎮楃紒顓㈢帛鐠併倖澧跨仦鏇熸煙閸氭垯鈧?
鐞涖儱鍘栫拠瀛樻閿?
- `launch / status / schema / load_robot / step / stop` 鏉╂瑦娼?lifecycle 瀹歌尙鎾奸崗銉ф埂鐎?headless smoke 妤犲本鏁归懠鍐ㄦ纯閵?- `2026-04-02` 閸?Windows 閺堫剙婀撮悳顖氼暔娑撳绱濇妯款吇閸︾儤娅?`demo_generated_biped.tscn` 娑?runner 閸︾儤娅?`run_rl_server.tscn` 闁棄鍑￠柅姘崇箖鏉╂瑦娼?lifecycle 妤犲本鏁归妴?
## 鐠嬪啳鐦楦款唴

1. 閸忓牏鈥樼拋銈呯唨绾偓閺堝秴濮熼崣顖欎簰閸氼垰濮╅獮鍓佹磧閸?`8000` 缁旑垰褰涢妴?2. 閸愬秷顔栭梻顕€顩绘い闈涙嫲闂堟瑦鈧線銆夐棃顫礉绾喛顓婚棃娆愨偓浣界カ濠ф劕濮炴潪鑺ヮ劀鐢悶鈧?3. 閺堚偓閸氬骸鍟€閹恒儱鍙?Godot 娴犺法婀＄粩顖ょ礉闁劖顒炵拫鍐槸 WebSocket 娑撳孩甯堕崚鑸靛复閸欙絻鈧?
## API 閸欏倽鈧?
### Godot 閹恒儱褰?
- `POST /api/godot/connect`: 鏉╃偞甯存禒璺ㄦ埂閸?- `POST /api/godot/start`: 閸氼垰濮╂禒璺ㄦ埂
- `POST /api/godot/stop`: 閸嬫粍顒涙禒璺ㄦ埂
- `POST /api/godot/update-params`: 閺囧瓨鏌婇悧鈺冩倞閸欏倹鏆?
- `GET /api/godot/capabilities`: 閺屻儳婀呰ぐ鎾冲閺€顖涘瘮閻?Godot 閹恒儱鍙嗗Ο鈥崇础

### Godot Agent 閹恒儱褰?
- `POST /execute`: 閹笛嗩攽 Godot Agent 閸楁洘娼崨鎴掓姢
- `POST /pipeline`: 閹笛嗩攽 Godot Agent 閸涙垝鎶ゅù浣规寜缁?- `GET /roles`: 閺屻儳婀呰ぐ鎾冲 backend 閻ㄥ嫯顫楅懝鑼叐闂?- `GET /api/godot-agent/templates`: 閸掓鍤ぐ鎾冲 backend 閺嗘挳婀堕惃鍕侀弶鑳カ濠?- `GET /api/godot-agent/templates/{template_id}`: 閼惧嘲褰囬崡鏇氶嚋濡剝婢樼拠锔藉剰
- `GET /api/godot_skills/list`: 閺冄冨悑鐎圭懓鍩嗛崥宥忕幢modern backend 娑撳绱伴幎鏇炲 templates
- `POST /api/godot_skills/apply`: 閺冄冨悑鐎圭懓鍩嗛崥宥忕幢modern backend 娑撳绱伴幎鏇炲閸楁洑閲?template
- `POST /api/godot-agent/plan`: 閻㈢喐鍨氶崣顖氱潔缁€铏规畱 Godot Agent 娴犺濮熺拋鈥冲灊
- `GET /api/godot-agent/history`: 閼惧嘲褰囬張鈧潻鎴滄崲閸斺€冲坊閸?- `GET /api/godot-agent/doctor`: 鏉╂劘顢?Godot Agent 閻滎垰顣ㄩ懛顏咁梾
- `POST /api/godot-agent/launch`: 鐠囬攱鐪伴崥顖氬З Godot 缂傛牞绶崳?
鐠囧瓨妲戦敍?
- `godot-agent` backend 閻ㄥ嫭顒滃蹇氱カ濠ф劘顕㈡稊澶屽箛閸︺劍妲?`templates`
- legacy `godot_studio_agent` backend 娴犲秳绻氶悾?`godot_skills`
- 娑撹桨绨℃稉宥嗗ⅵ閺傤厽妫崜宥囶伂閸滃本妫懘姘拱閿涘畭/api/godot_skills/*` 缂佈呯敾娣囨繄鏆€閿涘奔绲炬潻鏂挎礀闁插奔绱扮敮?`compatibility_alias` 閸?`source_kind`
- `/api/system/status` 閻滄澘婀导姘额杺婢舵牞绻戦崶?`godot_agent` 閹芥顩﹂敍宀勵浕妞ょ數濮搁幀浣稿幢娴兼氨娲块幒銉︽▔缁€鍝勭秼閸?backend閵嗕浇绁┃鎰佸蹇庝簰閸?roles/templates 鐠佲剝鏆?
- `godot-agent` modern backend 閻滄澘婀导姘▔瀵繒绮︾€规岸绮拋?`project_path` 閸?`history_file`閿涘矂浼╅崗宥呮躬閺堫亪鍘ょ純顕€銆嶉惄顔跨熅瀵板嫭妞傞崶鐐衡偓鈧崚?Web 鏉╂稓鈻艰ぐ鎾冲瀹搞儰缍旈惄顔肩秿

閸欘垶鈧俺绻冮悳顖氼暔閸欐﹢鍣洪崚鍥ㄥ床 backend閿?
- `AGI_WALKER_GODOT_AGENT_BACKEND=legacy|godot-agent`
- `AGI_WALKER_GODOT_AGENT_DIR=<agent_dir>`
- `AGI_WALKER_GODOT_PROJECT_PATH=<project_dir>`
- `AGI_WALKER_GODOT_AGENT_HISTORY_FILE=<history_file>`

姒涙顓荤悰灞艰礋閿?
- 婵″倹鐏夐張顏呮▔瀵繘鍘ょ純?`AGI_WALKER_GODOT_PROJECT_PATH`閿涘odern backend 娴兼岸绮拋銈勫▏閻劋绮ㄦ惔鎾冲敶閻?`godot_project/`
- 婵″倹鐏夐張顏呮▔瀵繘鍘ょ純?`AGI_WALKER_GODOT_AGENT_HISTORY_FILE`閿涘奔鎹㈤崝鈥冲坊閸欒弓绱版妯款吇閸愭瑥鍩?`.output/godot_agent_backend/task_history.json`
- 鐎电懓绨查崶鐐寸泊婢跺洣鍞ら惄顔肩秿姒涙顓绘担宥勭艾 `.output/godot_agent_backend/backups/`

`/api/system/status` 闁插瞼娈?`godot_agent` 閹芥顩﹂悳鏉挎躬鏉╂ü绱版潻鏂挎礀閿?
- `project_path`
- `history_file`

### 娴犺濮熼幒銉ュ經
- `GET /api/tasks`: 閼惧嘲褰囨禒璇插閸掓銆?
- `POST /api/tasks`: 閸掓稑缂撴禒璇插
- `DELETE /api/tasks/{id}`: 閸掔娀娅庢禒璇插

### Workflow 閹恒儱褰?
- `GET /api/workflows/`: 閸掓鍤?workflows
- `GET /api/workflows/{name}`: 閺屻儳婀?workflow 鐎规矮绠?
- `POST /api/workflows/{name}/run`: 閸氼垰濮╅崥搴″酱 workflow run
- `GET /api/workflows/runs`: 閺屻儴顕?runs 閸掓銆冮敍灞炬暜閹镐胶鐡柅澶堚偓浣稿瀻妞ら潧鎷拌ぐ鎺撱€傞懠鍐ㄦ纯
- `GET /api/workflows/runs/{run_id}`: 閼惧嘲褰囩€瑰本鏆?run 鐠囷附鍎?
- `GET /api/workflows/runs/{run_id}/status`: 閼惧嘲褰囨潪濠氬櫤閻樿埖鈧礁鎻╅悡?- `POST /api/workflows/runs/{run_id}/cancel`: 閸欐垿鈧礁褰囧☉鍫ｎ嚞濮?- `GET /api/workflows/runs/{run_id}/events`: 鐠併垽妲?SSE 娴滃娆㈠ù?- `GET /api/workflows/runs/{run_id}/live-log`: 閼惧嘲褰囩€瑰本鏆ｇ€圭偞妞傞弮銉ョ箶閺傚洦婀?
- `GET /api/workflows/runs/{run_id}/log`: 娑撳娴?workflow JSON log
- `GET /api/workflows/runs/{run_id}/artifacts/{artifact_index}`: 娑撳娴囬幐鍥х暰娴溠呭⒖
- `POST /api/workflows/runs/{run_id}/artifacts/{artifact_index}/godot-load`: 鐏?`robot_config` 娴溠呭⒖闁礁绶?Godot閿涘苯褰查柅?`legacy_controller` 閹?`session_bridge`
- `POST /api/workflows/runs/{run_id}/godot-sync`: 閼奉亜濮╅柅澶嬪閹恒劏宕?`robot_config` 娴溠呭⒖閿涘苯鑻熼幐澶婄暭閺傜鐭惧鍕偓浣哥窔 Godot

### 閻喎鐤?Godot Headless Smoke

娴犳挸绨遍悳鏉挎躬娣囨繄鏆€娴滃棔绔撮弶鈩冩▔瀵?opt-in 閻ㄥ嫮婀＄€?Godot headless smoke閿?
```bash
AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1 python -m pytest tests/test_godot_headless_smoke.py -q -m integration
```

姒涙顓?`python tests/run_smoke_tests.py` 娑撳秳绱板鍝勫煑鐠烘垼绻栭弶鈩冾梾閺屻儻绱遍崣顏呮箒閸︺劍妯夊蹇氼啎缂冾噯绱?

- `AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1`

閸氬函绱漵moke runner 閹靛秳绱版潻钘夊閻喎鐤?Godot headless 妤犲本鏁归妴?
婵″倹鐏夐棁鈧憰浣瑰Ω鏉╂瑦娼柧鎹愮熅瑜版挷缍旈悪顒傜彌 integration 閺嶅洤鍣禒鎯扮箥鐞涘矉绱濆楦款唴閺勬儳绱＄悰銉ょ瑐鐠囧﹥鏌囨禍褏澧块惄顔肩秿閿?
```powershell
$env:GODOT_EXECUTABLE='D:\鏉╁懘娴勬稉瀣祰\Godot\godot.EXE'
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
$env:AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR='test_env/godot_headless_smoke'
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

婵″倹鐏夋担鐘侯洣妤犲矁鐦?runner 閸︾儤娅欓敍灞肩瘍閸欘垯浜掗弰鎯х础閹稿洤鐣鹃敍?
```powershell
$env:GODOT_EXECUTABLE='D:\鏉╁懘娴勬稉瀣祰\Godot\godot.EXE'
$env:AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE='1'
$env:AGI_WALKER_GODOT_HEADLESS_SCENE='run_rl_server.tscn'
$env:AGI_WALKER_GODOT_HEADLESS_ARTIFACT_DIR='test_env/godot_headless_smoke_runner'
python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv
```

閸欘垵鐨熼崣鍌涙殶閸栧懏瀚敍?
- `AGI_WALKER_GODOT_HEADLESS_SCENE`
- `AGI_WALKER_GODOT_HEADLESS_CONNECT_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_SCHEMA_TIMEOUT_SECONDS`
- `AGI_WALKER_GODOT_HEADLESS_STEP_COUNT`

濮ｅ繑顐兼潻鎰攽闁垝绱伴崘娆忓毉缂佹挻鐎崠鏍槚閺傤厽鏋冩禒璁圭窗

- `test_env/godot_headless_smoke/headless_smoke_report.json`

閸忔湹鑵戞导姘愁唶瑜版洩绱?

- preflight 閻滎垰顣ㄧ紒鎾寸亯
- failure stage閿涙瓪environment / launch / tcp_connect / schema / load_robot / step_loop / teardown`
- Godot 鏉╂稓鈻肩拠濠冩焽娑?stdout/stderr tail
- schema 閹芥顩﹂崪?step 閺嶉攱婀?

`2026-04-02` 閻ㄥ嫭婀伴崷浼寸崣閺€璺烘彥閻撗嶇窗

- 閺勬儳绱＄拋鍓х枂 `GODOT_EXECUTABLE` 楠炶泛鎯庨悽?`AGI_WALKER_ENABLE_GODOT_HEADLESS_SMOKE=1` 閸氬函绱漙python tests/run_smoke_tests.py --output-root test_env/smoke_runs/v1_plan_check_after_fix` 瀹告煡鈧俺绻?
- `python -m pytest tests/test_godot_headless_smoke.py -q -m integration --tb=short -vv` 瀹告煡鈧俺绻?
- 鐠佸墽鐤?`AGI_WALKER_GODOT_HEADLESS_SCENE=run_rl_server.tscn` 閸氬骸顦茬捄鎴濇倱娑撯偓閺?lifecycle smoke閿涘奔绡冨鏌モ偓姘崇箖

CI 娑擃厼褰熸径鏍︾箽閻ｆ瑤绨℃稉鈧稉?`godot-headless-smoke` job閿涘苯鐣犻悳鏉挎躬閺€顖涘瘮閿?
- `workflow_dispatch`
- nightly schedule

閸氬本鐗遍敍瀹峝istributed-smoke` 娑旂喎鍑℃潻娑樺弳 nightly閵嗕糠ightly 娑撳秳绱版潻鎰攽鐎瑰本鏆?unit/integration 閻晠妯€閿涘矁鈧本妲告稉鎾绘，娣囨繄鏆€妤傛ü鐜崐鑲╂畱娑撴捇銆嶉柧鎹愮熅閸ョ偛缍婇妴?
