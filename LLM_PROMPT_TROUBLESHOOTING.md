# LLM Prompt Troubleshooting Guide

Common issues when using natural language prompts and how to fix them.

---

## Issue 1: Robot Placed in Wrong Location

**Symptom**: "Place Kinova on the table" → Robot appears on the shelf instead

**Root Cause**: LLM created wrong constraint reference or wrong constraint type

**Example of Wrong Output**:
```json
{
  "robot_id": "kinova",
  "robot_type": "kinova_gen3",
  "constraints": [
    {"type": "beside", "subject": "kinova", "reference": "shelf", ...}  // WRONG!
  ]
}
```

**Correct Output**:
```json
{
  "robot_id": "kinova",
  "robot_type": "kinova_gen3",
  "constraints": [
    {"type": "on_top_of", "subject": "kinova", "reference": "table", "clearance": 0.001}  // ✓
  ]
}
```

**Fix**: Added Example 6 showing robot with `on_top_of` constraint

---

## Issue 2: Numerical Parameters Not Extracted

**Symptom**: "5 shelves, 2m tall" → Gets default 2 shelves, 0.9m tall

**Root Cause**: LLM didn't extract numbers from natural language

**Solution**: Use explicit numerical format in prompt:
- ❌ "a shelf with 5 layers"
- ✓ "a shelf with num_shelves: 5, height: 1.5m"

**Fix**: Added guideline #9 and Example 5 showing number extraction

---

## Issue 3: Spatial Directions Misunderstood

**Symptom**: "to the left of table" → Appears beside (+Y) instead of left (-X)

**Root Cause**: LLM doesn't know which constraint/offset to use

**Spatial Language Mapping**:
- "on X" → `on_top_of` constraint
- "to the left of X" → `beside` with `offset: [-1.0, 0, 0]`
- "to the right of X" → `beside` with `offset: [1.0, 0, 0]`
- "in front of X" → `in_front_of` (positive clearance)
- "behind X" → `in_front_of` with negative clearance or offset
- "next to X" / "beside X" → `beside` (default +Y direction)
- "opposite side" → Use large offset to go around object

**Fix**: Added guideline #10 with explicit direction mappings

---

## Issue 4: Wrong Number of Shelves Generated

**Symptom**: Asked for 5 shelves, got 3

**Investigation**:
```python
# Check what LLM generated
dimensions = {
  "width": 0.6,
  "depth": 0.3,
  "height": 1.5,  # Did LLM use this?
  "num_shelves": 5  # Or default 2?
}
```

**Debugging**:
1. Check the JSON scene description LLM generated (before XML)
2. Log: `self.last_structured_json` to see what LLM actually created
3. Verify `num_shelves` is in dimensions dict

**Common Causes**:
- LLM used default (2) instead of extracting 5
- LLM confused "5 layers" with "5 interior shelves" 
- Validation stripped the parameter

**Solution**: Be more explicit in prompt:
```
"Create a shelf unit with dimensions: {width: 0.6, depth: 0.3, height: 1.5, num_shelves: 5}"
```

---

## Issue 5: Shelf Height Calculation

**User Says**: "5 layers with 0.3m each" 

**Ambiguity**:
- Does this mean 5 interior shelves? → `num_shelves: 5`, `height: ?`
- Or total height = 5 × 0.3 = 1.5m? → `num_shelves: ?`, `height: 1.5`
- Or each shelf spacing is 0.3m? → `num_shelves: 5`, `height: 1.5m`

**Recommendation**: Be explicit:
```
"Create a 1.5m tall shelf with 5 interior shelf levels"
→ dimensions: {width: 0.6, depth: 0.3, height: 1.5, num_shelves: 5}
```

---

## Best Practices for Reliable LLM Scene Generation

### 1. Use Explicit Dimensions
❌ "a big shelf"
✓ "a shelf 0.8m wide, 1.5m tall"

### 2. Use Clear Spatial Language
❌ "near the table"
✓ "to the left of the table" or "0.5m to the left of the table"

### 3. One Relationship Per Sentence
❌ "Put shelf left of table and robot on opposite side"
✓ "Put shelf to left of table. Put robot to right of table."

### 4. Specify All Constraint References Explicitly
❌ "Place robot there"
✓ "Place robot on the table"

### 5. Use Named Configurations When Available
```
"Place Kinova Gen3 in ready pose on the table"
→ joint_config: "ready"
```

---

## Debug Checklist

When LLM generates incorrect scenes:

1. ✓ Check `last_structured_json` - What did LLM actually generate?
2. ✓ Verify all `constraint.reference` IDs exist
3. ✓ Check numerical parameters were extracted
4. ✓ Verify spatial constraints match intent
5. ✓ Check object order (supports before dependents)

**Access Debug Info**:
```python
generator = LLMSceneGenerator(metadata_extractor)
trace = generator.generate_scene_with_trace(prompt)

print("LLM Raw JSON:", trace['llm_raw_json'])
print("Structured Scene:", trace['structured_scene_json'])
print("Constraint Fixes:", trace['constraint_fixes'])
```

---

## Quick Fixes

### Problem: Robot in wrong place
**Quick Fix**: Add explicit spatial constraint in prompt
```
"Place kinova_gen3 robot on the table (use on_top_of constraint)"
```

### Problem: Wrong dimensions
**Quick Fix**: Use JSON-like syntax in prompt
```
"Create shelf with {width: 0.8, depth: 0.3, height: 2.0, num_shelves: 5}"
```

### Problem: Objects colliding
**Quick Fix**: Add explicit clearance
```
"Place objects with 0.1m clearance between them"
```

---

## Testing Your Prompts

Before using in production, test with:

```python
from mujoco_mcp.scene_gen import LLMSceneGenerator, MetadataExtractor

metadata = MetadataExtractor()
generator = LLMSceneGenerator(metadata)

# Test your prompt
prompt = "Put a shelf with 5 layers, 2m tall, left of table. Kinova on table."
trace = generator.generate_scene_with_trace(prompt)

# Inspect what LLM generated
print("Objects:", len(trace['scene_description'].objects))
print("Robots:", len(trace['scene_description'].robots))

# Check specific dimensions
for obj in trace['scene_description'].objects:
    if 'shelf' in obj.object_id:
        print(f"Shelf dimensions: {obj.dimensions}")
```

---

**Last Updated**: October 5, 2025  
**Related Docs**: 
- [INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)
- [ADDING_NEW_OBJECTS_GUIDE.md](ADDING_NEW_OBJECTS_GUIDE.md)
