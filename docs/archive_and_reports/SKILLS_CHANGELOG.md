# AGI-Walker Skills 缁崵绮洪弴瀛樻煀閺冦儱绻?

## 2026-02-06: Moltbot Skills 閺佹潙鎮庣€瑰本鍨?

### 閺傛澘顤冮崝鐔诲厴

####  Skills 缁崵绮哄鍡樼仸
- 閸掓稑缂?`.agent/AGENTS.md` 妞ゅ湱娲扮憴鍕瘱閺傚洦銆?
- 鐎圭偟骞?`agi_walker/skills_loader.py` 閺嶇绺鹃崝鐘烘祰閸?
  - SkillMetadata 閺佺増宓佺猾?
  - SkillsLoader 娑撹崵琚?(閺€顖涘瘮閹兼粎鍌?閸掑棛琚?娓氭繆绂嗘宀冪槈)
  - YAML frontmatter 鐟欙絾鐎?
  - 濞撴劘绻樺蹇撳鏉炶姤婧€閸?
- 濞ｈ濮炵€瑰本鏆ｉ崡鏇炲帗濞村鐦總妞炬

#### 棣冾樆 Robot Modeling Skill
- 鐎圭偟骞?RobotBuilder 濞翠礁绱PI
  - `add_torso()` - 濞ｈ濮為煬顖氬叡
  - `add_leg_pair()` - 濞ｈ濮為懙鍨嚠
  - `add_arm_pair()` - 濞ｈ濮為幍瀣櫐鐎?
  - `set_joint_damping()` - 鐠佸墽鐤嗛梼璇插嚬
  - `set_joint_limits()` - 鐠佸墽鐤嗛梽鎰秴
  - `customize()` - 閼奉亜鐣炬稊澶婂棘閺?
- 閸掓稑缂?RobotConfig 闁板秶鐤嗙猾?
- 鐎圭偟骞囧Ο鈩冩緲閸旂姾娴囬崳?`load_template()`
- 濞ｈ濮?娑擃亪顣╃拋鐐侀弶?
  - `biped_basic` - 閸╄櫣顢呴崣宀冨喕閺堝搫娅掓禍?
  - `quadruped_dog` - 閸ユ稖鍐婚悩顒€鑸伴張鍝勬珤娴?
- 缂傛牕鍟撶€瑰本鏆?API 閸欏倽鈧啯鏋冨?

### 閹垛偓閺堫垯瀵掗悙?

1. **濡€虫健閸栨牞顔曠拋?*: Skills 閻欘剛鐝涚亸浣筋棅,閺勬挷绨幍鈺佺潔
2. **濞撴劘绻樺蹇撳鏉?*: 娴犲懎婀棁鈧憰浣规閸旂姾娴囩€瑰本鏆ｉ弬鍥ㄣ€?闁灝鍘ゆ稉濠佺瑓閺傚洦钖勯弻?
3. **濞翠礁绱PI**: 閺€顖涘瘮闁炬儳绱＄拫鍐暏,娴狅絿鐖滅粻鈧ú浣风喘闂?
4. **濡剝婢樼化鑽ょ埠**: 妫板嫯顔曞Ο鈩冩緲闂勫秳缍嗘担璺ㄦ暏闂傘劍顫?

### 娴ｈ法鏁ょ粈杞扮伐

```python
from agi_walker.skills.robot_modeling import RobotBuilder

# 閸掓稑缂撻張鍝勬珤娴?
robot = (
    RobotBuilder("my_biped")
    .add_torso(height=0.5, mass=5.0)
    .add_leg_pair(thigh_length=0.3, shin_length=0.3)
    .set_joint_damping(0.5)
    .build()
)

# 娣囨繂鐡ㄩ柊宥囩枂
robot.save("configs/my_biped.json")
```

### 娑撳绔村銉吀閸?

闂冭埖顔? - 閺嶇绺?Skills 瀵偓閸?
- [ ] Parameter Optimizer Skill (閸欏倹鏆熸导妯哄)
- [ ] URDF Generator Skill (閺嶇厧绱℃潪顒佸床)
- [ ] Simulation Runner Skill (娴犺法婀￠幍褑顢?

闂冭埖顔? - AI Agent 閺佹潙鎮?
- [ ] GUI Skills 闂堛垺婢?
- [ ] CLI skills 閸涙垝鎶?
- [ ] 鐎瑰本鏆ｉ弬鍥ㄣ€傞崪宀€銇氭笟?

---

**缂佸瓨濮㈤懓?*: AGI-Walker 瀵偓閸欐垵娲熼梼?
**閺佹潙鎮庨悘鍨妳**: Moltbot Skills 缁崵绮?
