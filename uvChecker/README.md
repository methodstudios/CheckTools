## UVChecker
Find general uv errors

### Check numbers
0. Udim border intersections
1. Non-mapped UV faces
2. Zero-area Uv faces
3. Unassigned UVs
4. UVs in negative space

### Flags
| Longname | Shortname | Argument types | Default | Properties |
|:---------|----------:|:--------------:|:-------:|:----------:|
|check|c|integer||C|
|uvArea|uva|double|0.000001|C|
|verbose|v|bool|False|C|

### Example
```python
from maya import cmds
errors = cmds.checkUV("|pSphere1", c=0)
print errors
>>> [u'|pSphere1|pSphereShape1.map[19]', u'|pSphere1|pSphereShape1.map[20]', ...]
```