# Skills 瀵偓閸欐垶瀵氶崡?

閺堫剚瀵氶崡妤€鐨㈢敮顔煎И娴ｇ姳璐烝GI-Walker瀵偓閸欐垶鏌婇惃鍑穔ills閵?

---

## 韫囶偊鈧喎绱戞慨?

### 1. 閸掓稑缂揝kill閻╊喖缍?

```bash
mkdir -p agi_walker/skills/my-skill
cd agi_walker/skills/my-skill
```

### 2. 閸掓稑缂揝KILL.md

閸掓稑缂?`SKILL.md` 閺傚洣娆?閸栧懎鎯圷AML frontmatter閸滃arkdown閸愬懎顔?

```markdown
---
name: my-skill
description: "缁犫偓閻厽寮挎潻棰佺稑閻ㄥ墕kill閸旂喕鍏橀崪宀勨偓鍌滄暏閸︾儤娅?
metadata:
  agi_walker:
    emoji: "棣冩暋"
    category: "瀹搞儱鍙?
    requires:
      python_modules: ["numpy"]
---

# My Skill

鐠囷妇绮忛惃鍕▏閻劍鏋冨?..

## 韫囶偊鈧喎绱戞慨?

...

## API閸欏倽鈧?

...
```

### 3. 鐎圭偟骞囬弽绋跨妇閸旂喕鍏?

閸掓稑缂?`__init__.py`:

```python
"""
My Skill - 閸旂喕鍏橀幓蹇氬牚
"""

def my_function():
    """娑撴槒顩﹂崝鐔诲厴"""
    pass
```

### 4. 濞村鐦?

```bash
python -m agi_walker.cli skills info my-skill
```

---

## SKILL.md 鐟欏嫯瀵?

### Frontmatter 鐎涙顔?

#### 韫囧懘娓剁€涙顔?

```yaml
name: skill-name          # Skill閸氬秶袨 (kebab-case)
description: "閹诲繗鍫?.."    # 缁犫偓閻厽寮挎潻?(娑撯偓閸欍儴鐦?
```

#### 閸欘垶鈧鐡у▓?

```yaml
metadata:
  agi_walker:
    emoji: "棣冩暋"            # Emoji閸ョ偓鐖?
    category: "閸掑棛琚?       # 閸掑棛琚崥宥囆?
    requires:
      python_modules:      # Python娓氭繆绂?
        - numpy
        - scipy
      bins:                # 閸涙垝鎶ょ悰灞戒紣閸忚渹绶风挧?
        - ffmpeg
      system_packages:     # 缁崵绮洪崠鍛贩鐠?
        - libopencv-dev
```

### Markdown 閸愬懎顔愮紒鎾寸€?

閹恒劏宕樼紒鎾寸€?

```markdown
# Skill閸氬秶袨

缁犫偓娴?(1-2濞?

## 韫囶偊鈧喎绱戞慨?

閺堚偓缁犫偓閸楁洜娈戞担璺ㄦ暏缁€杞扮伐 (娴狅絿鐖滈崸?

## 閸旂喕鍏橀悧瑙勨偓?

- 閻楄鈧?
- 閻楄鈧?

## API閸欏倽鈧?

鐠囷妇绮廇PI鐠囧瓨妲?(閹存牠鎽奸幒銉ュ煂 references/api.md)

## 缁€杞扮伐

鐎瑰本鏆ｆ担璺ㄦ暏缁€杞扮伐

## 鐢瓕顫嗛梻顕€顣?

Q&A

## 娑撳绔村?

閻╃鍙kill閹恒劏宕?
```

---

## 閻╊喖缍嶇紒鎾寸€?

### 閺堚偓鐏忓繒绮ㄩ弸?

```
my-skill/
閳规壕鏀㈤埞鈧?SKILL.md           # 韫囧懘娓? Skill閺傚洦銆?
閳规柡鏀㈤埞鈧?__init__.py        # 韫囧懘娓? 閺嶇绺剧€圭偟骞?
```

### 鐎瑰本鏆ｇ紒鎾寸€?

```
my-skill/
閳规壕鏀㈤埞鈧?SKILL.md                  # Skill閺傚洦銆?
閳规壕鏀㈤埞鈧?__init__.py              # 閺嶇绺剧€圭偟骞?
閳规壕鏀㈤埞鈧?scripts/                 # 閸欘垶鈧? 閸涙垝鎶ょ悰灞戒紣閸?
閳?  閳规壕鏀㈤埞鈧?tool1.py
閳?  閳规柡鏀㈤埞鈧?tool2.py
閳规壕鏀㈤埞鈧?references/              # 閸欘垶鈧? 閸欏倽鈧啯鏋冨?
閳?  閳规壕鏀㈤埞鈧?api.md
閳?  閳规柡鏀㈤埞鈧?algorithms.md
閳规壕鏀㈤埞鈧?assets/                  # 閸欘垶鈧? 鐠у嫭绨弬鍥︽
閳?  閳规壕鏀㈤埞鈧?templates/
閳?  閳规柡鏀㈤埞鈧?examples/
閳规柡鏀㈤埞鈧?tests/                   # 閸欘垶鈧? 濞村鐦?
    閳规柡鏀㈤埞鈧?test_my_skill.py
```

---

## 娴狅絿鐖滅憴鍕瘱

### 鐎电厧鍤幒銉ュ經

閸?`__init__.py` 娑擃厽妲戠涵顔碱嚤閸?

```python
"""
My Skill - 閸旂喕鍏橀幓蹇氬牚
"""

from .core import MyClass, my_function

__all__ = ['MyClass', 'my_function']
```

### 娓氭寧宓庨崙鑺ユ殶

閹绘劒绶电粻鈧崡鏇犳畱妞よ泛鐪伴崙鑺ユ殶:

```python
def do_something(input_file: str, output_file: str) -> Result:
    """缁犫偓濞蹭胶娈慉PI
    
    Args:
        input_file: 鏉堟挸鍙嗛弬鍥︽
        output_file: 鏉堟挸鍤弬鍥︽
        
    Returns:
        Result鐎电钖?
        
    Example:
        >>> result = do_something("in.json", "out.json")
        >>> print(result.success)
        True
    """
    # 鐎圭偟骞?..
```

### 缁鐎峰▔銊ㄐ?

娴ｈ法鏁ょ猾璇茬€峰▔銊ㄐ?

```python
from typing import Dict, List, Optional
from pathlib import Path

def process_data(
    data: Dict[str, Any],
    options: Optional[List[str]] = None
) -> Path:
    ...
```

---

## 閸涙垝鎶ょ悰灞戒紣閸?

### 閸掓稑缂揅LI閼存碍婀?

閸?`scripts/` 閻╊喖缍嶉崚娑樼紦:

```python
#!/usr/bin/env python
"""
My Tool - 閹诲繗鍫?
"""

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

from agi_walker.skills.my_skill import do_something


def main():
    parser = argparse.ArgumentParser(description="...")
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    
    args = parser.parse_args()
    
    result = do_something(args.input, args.output)
    
    if result.success:
        print(f"閴?閹存劕濮?)
        return 0
    else:
        print(f"閴?婢惰精瑙? {result.error}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
```

---

## 閸欏倽鈧啯鏋冨?

### API閺傚洦銆傚Ο鈩冩緲

閸?`references/api.md`:

```markdown
# My Skill API 閸欏倽鈧?

## 閸戣姤鏆?

### do_something

\`\`\`python
do_something(input_file: str, output_file: str) -> Result
\`\`\`

閹诲繗鍫?..

**閸欏倹鏆?**
- `input_file` - 鐠囧瓨妲?
- `output_file` - 鐠囧瓨妲?

**鏉╂柨娲?**
- `Result` - 鐠囧瓨妲?

**缁€杞扮伐:**
\`\`\`python
result = do_something("in.json", "out.json")
\`\`\`

## 缁?

### MyClass

...
```

---

## 閺堚偓娴ｅ啿鐤勭捄?

### 1. 濞撴劘绻樺蹇斿Л闂?

闁潧鎯奙oltbot閻ㄥ嫭绗庢潻娑樼础閹额偊婀堕崢鐔峰灟:

- **Level 1 (Metadata)**: 缁犫偓閻厾娈憂ame閸滃畳escription
- **Level 2 (SKILL.md)**: 韫囶偊鈧喎绱戞慨瀣嫲閺嶇绺鹃崝鐔诲厴
- **Level 3 (References)**: 鐠囷妇绮廇PI閸滃瞼鐣诲▔鏇熸瀮濡?

### 2. 缁€杞扮伐娴兼ê鍘?

閹绘劒绶甸崣顖濈箥鐞涘瞼娈戠粈杞扮伐:

```python
# 閴?娑撳秴銈?
def process(data): ...

# 閴?婵?
def process(data):
    """
    Example:
        >>> from agi_walker.skills.my_skill import process
        >>> result = process({"key": "value"})
        >>> print(result)
        {'processed': True}
    """
    ...
```

### 3. 闁挎瑨顕ゆ径鍕倞

閹绘劒绶靛〒鍛珰閻ㄥ嫰鏁婄拠顖欎繆閹?

```python
if not input_path.exists():
    raise FileNotFoundError(
        f"鏉堟挸鍙嗛弬鍥︽娑撳秴鐡ㄩ崷? {input_path}\n"
        f"鐠囬攱顥呴弻銉ㄧ熅瀵板嫭妲搁崥锔筋劀绾?
    )
```

### 4. 閺傚洦銆傜€涙顑佹稉?

娴ｈ法鏁oogle妞嬪孩鐗竏ocstring:

```python
def my_function(param1: str, param2: int = 0) -> bool:
    """缁犫偓閻厽寮挎潻?
    
    鐠囷妇绮忕拠瀛樻 (閸欘垶鈧?
    
    Args:
        param1: 閸欏倹鏆?鐠囧瓨妲?
        param2: 閸欏倹鏆?鐠囧瓨妲? 姒涙顓?
        
    Returns:
        閹存劕濮涙潻鏂挎礀True
        
    Raises:
        ValueError: 婵″倹鐏塸aram1娑撹櫣鈹?
        
    Example:
        >>> my_function("test")
        True
    """
    ...
```

---

## 閸掑棛琚幐鍥у础

### 瀵ら缚顔呴惃鍕瀻缁?

- **瀵ょ儤膩** - 閺堝搫娅掓禍鍝勭紦濡紕娴夐崗?
- **娴兼ê瀵?* - 閸欏倹鏆熸导妯哄閻╃鍙?
- **鏉烆剚宕?* - 閺嶇厧绱℃潪顒佸床閻╃鍙?
- **娴犺法婀?* - 娴犺法婀￠幍褑顢戦惄绋垮彠
- **閺佺増宓侀悽鐔稿灇** - AI閺佺増宓侀悽鐔稿灇
- **閸掑棙鐎?* - 閺佺増宓侀崚鍡樼€藉銉ュ徔
- **瀹搞儱鍙?* - 闁氨鏁ゅ銉ュ徔
- **閸忔湹绮?* - 閸忔湹绮猾璇茬€?

### 闁瀚ㄩ崚鍡欒

閺嶈宓丼kill閻ㄥ嫪瀵岀憰浣烘暏闁棃鈧瀚?
- 婵″倹鐏塖kill閺堝顦挎稉顏嗘暏闁?闁瀚ㄩ張鈧稉鏄忣洣閻?
- 鐏忎粙鍣烘担璺ㄦ暏瀹稿弶婀侀崚鍡欒
- 韫囧懓顩﹂弮璺哄讲娴犮儱鍨卞鐑樻煀閸掑棛琚?

---

## 娓氭繆绂嗙粻锛勬倞

### 婢圭増妲戞笟婵婄

閸︹娍KILL.md frontmatter娑擃厼锛愰弰?

```yaml
metadata:
  agi_walker:
    requires:
      python_modules:
        - numpy>=1.24.0
        - scipy>=1.10.0
      bins:
        - ffmpeg
```

### 濡偓閺屻儰绶风挧?

閸︺劋鍞惍浣疯厬濡偓閺?

```python
def __init__(self):
    try:
        import numpy as np
    except ImportError:
        raise ImportError(
            "This skill requires numpy.\n"
            "Install: pip install numpy>=1.24.0"
        )
```

---

## 濞村鐦?

### 閸楁洖鍘撳ù瀣槸

閸掓稑缂?`tests/test_my_skill.py`:

```python
import pytest
from agi_walker.skills.my_skill import do_something


def test_basic_functionality():
    """濞村鐦崺鐑樻拱閸旂喕鍏?""
    result = do_something("test.json", "out.json")
    assert result.success


def test_error_handling():
    """濞村鐦柨娆掝嚖婢跺嫮鎮?""
    with pytest.raises(FileNotFoundError):
        do_something("nonexistent.json", "out.json")
```

### 鏉╂劘顢戝ù瀣槸

```bash
pytest tests/test_my_skill.py -v
```

---

## 閸欐垵绔峰〒鍛礋

閹绘劒姘﹂弬鐧漦ill閸撳秵顥呴弻?

- [ ] SKILL.md閸栧懎鎯堢€瑰本鏆rontmatter
- [ ] 閺堝鎻╅柅鐔风磻婵銇氭笟?
- [ ] 娴狅絿鐖滈張澶岃閸ㄥ鏁炵憴?
- [ ] 娑撴槒顩﹂崙鑺ユ殶閺堝—ocstring
- [ ] 閺堝濞囬悽銊с仛娓?
- [ ] 婢圭増妲戞禍鍡樺閺堝绶风挧?
- [ ] 闁俺绻?`agi_walker skills validate`
- [ ] 閸︹€揕I閸滃瓘UI娑擃厽绁寸拠鏇＄箖

---

## 缁€杞扮伐

閸欏倽鈧啰骞囬張濉杒ills:

- `robot-modeling` - 鐎瑰本鏆ｉ惃鍕ウ瀵粐PI缁€杞扮伐
- `parameter-optimizer` - 缁夋垵顒熺拋锛勭暬闂嗗棙鍨氱粈杞扮伐
- `urdf-generator` - 閺嶇厧绱℃潪顒佸床缁€杞扮伐

---

## 閼惧嘲褰囩敮顔煎И

- 閺屻儳婀?`.agent/AGENTS.md` - 妞ゅ湱娲扮憴鍕瘱
- 闂冨懓顕伴悳鐗堟箒Skills閻ㄥ嫭绨惍?
- 娴ｈ法鏁?`agi_walker skills info <name> -d` 閺屻儳婀呴弬鍥ㄣ€?

---

**鐠侀缍?*: Skills鎼存棁顕氱粻鈧ú浣碘偓浣哥杽閻劊鈧焦鏋冨锝夌秷閸忋劊鈧倷绱崗鍫熷絹娓氭稑褰查悽銊ф畱瀹搞儱鍙?閼板矂娼径宥嗘絽閻ㄥ嫭濞婄挒掳鈧?
