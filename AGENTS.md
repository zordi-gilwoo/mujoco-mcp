# Agent-Driven Scene Generation Enhancement Plan

## Current System Assessment

### Status: Working but Limited

The LLM scene generation system is now functional and successfully:
- ✅ Connects to OpenAI API (gpt-4)
- ✅ Generates structured JSON from natural language
- ✅ Validates constraint references
- ✅ Converts to MuJoCo XML

**However**, it has critical limitations that prevent it from handling realistic use cases.

---

## Current Limitations

### 1. Very Limited Asset Database

Only **4 object types** available:
- `table_standard` - Fixed size: 0.8m × 1.2m × 0.75m
- `cup_ceramic_small` - Fixed size: 0.08m diameter, 0.1m height
- `box_small` - Fixed size: 0.1m × 0.1m × 0.1m
- `shelf_small` - Fixed dimensions

**Problem**: LLM can only work with these 4 objects, making it impossible to create most real-world scenarios.

### 2. No Parametric Primitives

All objects use XML templates with **hardcoded dimensions**:
```xml
<!-- Example from assets_db.json -->
<body name="{name}" pos="{pos}" quat="{quat}">
  <geom name="{name}_geom" type="box" size="0.05 0.05 0.05" rgba="0.7 0.2 0.2 1"/>
</body>
```

**Problem**: When user requests "2m long pole", the system can only use `box_small` with fixed 0.1m dimensions.

### 3. Fixed Schema Without Dimension Parameters

Current `ObjectPlacement` schema (`scene_schema.py`):
```python
class ObjectPlacement(BaseModel):
    object_id: str
    object_type: str  # Only references predefined asset types
    constraints: List[SpatialConstraint]
    orientation: Optional[Tuple[float, float, float, float]]
    # ❌ No way to specify custom dimensions
```

**Problem**: Even if LLM wants to specify custom sizes, the schema doesn't support it.

### 4. LLM Prompt Doesn't Mention Primitives

System prompt (lines 716-724 in `llm_scene_generator.py`) only lists the 4 assets:
```
## Available Assets:
**Objects:**
- table_standard: Standard work table (0.8m x 1.2m x 0.05m)
- cup_ceramic_small: Small ceramic cup (0.08m diameter, 0.1m height)
- box_small: Small cardboard box (0.1m x 0.1m x 0.1m)
- shelf_small: Small storage shelf with multiple levels
```

**Problem**: LLM has no knowledge that it could create spheres, cylinders, custom-sized boxes, etc.

---

## Recommended Enhancement Plan

### Phase 1: Core Parametric Primitive Support

#### 1.1 Extend Scene Schema

**File**: `src/mujoco_mcp/scene_gen/scene_schema.py`

Add dimension and appearance parameters to `ObjectPlacement`:

```python
class ObjectPlacement(BaseModel):
    object_id: str
    object_type: str  # Can be "primitive:box", "primitive:sphere", etc.
    constraints: List[SpatialConstraint]
    orientation: Optional[Tuple[float, float, float, float]]
    
    # NEW: Parametric dimensions
    dimensions: Optional[Dict[str, float]] = Field(
        default=None,
        description="Custom dimensions in meters (e.g., {'width': 0.5, 'height': 2.0})"
    )
    
    # NEW: Custom appearance
    color: Optional[Tuple[float, float, float, float]] = Field(
        default=None,
        description="RGBA color values [0-1] (e.g., (0.8, 0.2, 0.2, 1.0))"
    )
    
    @model_validator(mode="after")
    def validate_primitive_dimensions(self):
        """Validate that primitives have required dimensions."""
        if self.object_type.startswith("primitive:"):
            primitive_type = self.object_type.split(":")[1]
            
            if not self.dimensions:
                raise ValueError(f"Primitive type {self.object_type} requires dimensions")
            
            # Validate required keys based on primitive type
            required = {
                "box": {"width", "depth", "height"},
                "sphere": {"radius"},
                "cylinder": {"radius", "height"},
                "capsule": {"radius", "length"},
                "ellipsoid": {"radius_x", "radius_y", "radius_z"}
            }
            
            if primitive_type in required:
                missing = required[primitive_type] - set(self.dimensions.keys())
                if missing:
                    raise ValueError(
                        f"Primitive {primitive_type} missing dimensions: {missing}"
                    )
        
        return self
```

#### 1.2 Add Primitive Types to Asset Database

**File**: `src/mujoco_mcp/scene_gen/assets_db.json`

Add parametric primitive definitions:

```json
{
  "primitive:box": {
    "type": "primitive",
    "primitive_shape": "box",
    "description": "Parametric box with custom dimensions",
    "required_dimensions": ["width", "depth", "height"],
    "default_dimensions": {
      "width": 0.1,
      "depth": 0.1,
      "height": 0.1
    },
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"box\" size=\"{size}\" rgba=\"{rgba}\"/></body>"
  },
  "primitive:sphere": {
    "type": "primitive",
    "primitive_shape": "sphere",
    "description": "Parametric sphere with custom radius",
    "required_dimensions": ["radius"],
    "default_dimensions": {
      "radius": 0.05
    },
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"sphere\" size=\"{size}\" rgba=\"{rgba}\"/></body>"
  },
  "primitive:cylinder": {
    "type": "primitive",
    "primitive_shape": "cylinder",
    "description": "Parametric cylinder with custom radius and height",
    "required_dimensions": ["radius", "height"],
    "default_dimensions": {
      "radius": 0.05,
      "height": 0.1
    },
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"cylinder\" size=\"{size}\" rgba=\"{rgba}\"/></body>"
  },
  "primitive:capsule": {
    "type": "primitive",
    "primitive_shape": "capsule",
    "description": "Parametric capsule (rounded cylinder)",
    "required_dimensions": ["radius", "length"],
    "default_dimensions": {
      "radius": 0.05,
      "length": 0.2
    },
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"capsule\" size=\"{size}\" rgba=\"{rgba}\"/></body>"
  },
  "primitive:ellipsoid": {
    "type": "primitive",
    "primitive_shape": "ellipsoid",
    "description": "Parametric ellipsoid with three radii",
    "required_dimensions": ["radius_x", "radius_y", "radius_z"],
    "default_dimensions": {
      "radius_x": 0.05,
      "radius_y": 0.05,
      "radius_z": 0.05
    },
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"ellipsoid\" size=\"{size}\" rgba=\"{rgba}\"/></body>"
  }
}
```

#### 1.3 Update XML Builder for Parametric Primitives

**File**: `src/mujoco_mcp/scene_gen/scene_xml_builder.py`

Modify `_add_object_to_worldbody()` method:

```python
def _add_object_to_worldbody(
    self, 
    worldbody: ET.Element, 
    object_id: str, 
    object_type: str, 
    pose: Pose,
    dimensions: Optional[Dict[str, float]] = None,
    color: Optional[Tuple[float, float, float, float]] = None
):
    """Add an object to the worldbody with optional custom dimensions."""
    
    metadata = self.metadata_extractor.get_metadata(object_type)
    
    if metadata and metadata.xml_template:
        # Check if this is a parametric primitive
        if object_type.startswith("primitive:"):
            size_str = self._compute_primitive_size(object_type, dimensions, metadata)
            rgba_str = self._format_rgba(color) if color else "0.7 0.7 0.7 1"
        else:
            # Fixed-size asset from template
            size_str = ""  # Not used for fixed assets
            rgba_str = ""
        
        # Format template
        formatted_xml = metadata.xml_template.format(
            name=object_id,
            pos=f"{pose.position[0]:.6f} {pose.position[1]:.6f} {pose.position[2]:.6f}",
            quat=f"{pose.orientation[0]:.6f} {pose.orientation[1]:.6f} {pose.orientation[2]:.6f} {pose.orientation[3]:.6f}",
            size=size_str,
            rgba=rgba_str
        )
        
        try:
            object_element = ET.fromstring(formatted_xml)
            worldbody.append(object_element)
            logger.debug(f"Added object {object_id} using template")
        except ET.ParseError as e:
            logger.warning(f"Failed to parse XML template for {object_id}: {e}")
            self._add_fallback_object(worldbody, object_id, object_type, pose)
    else:
        self._add_fallback_object(worldbody, object_id, object_type, pose)

def _compute_primitive_size(
    self, 
    object_type: str, 
    dimensions: Optional[Dict[str, float]], 
    metadata: AssetMetadata
) -> str:
    """Compute MuJoCo size string for primitive types."""
    
    primitive_shape = object_type.split(":")[1]
    dims = dimensions or metadata.get_dimensions()
    
    if primitive_shape == "box":
        # MuJoCo box size = half-extents
        return f"{dims['width']/2:.6f} {dims['depth']/2:.6f} {dims['height']/2:.6f}"
    
    elif primitive_shape == "sphere":
        return f"{dims['radius']:.6f}"
    
    elif primitive_shape == "cylinder":
        # MuJoCo cylinder size = [radius, half-height]
        return f"{dims['radius']:.6f} {dims['height']/2:.6f}"
    
    elif primitive_shape == "capsule":
        # MuJoCo capsule size = [radius, half-length]
        return f"{dims['radius']:.6f} {dims['length']/2:.6f}"
    
    elif primitive_shape == "ellipsoid":
        return f"{dims['radius_x']:.6f} {dims['radius_y']:.6f} {dims['radius_z']:.6f}"
    
    else:
        logger.warning(f"Unknown primitive shape: {primitive_shape}")
        return "0.05 0.05 0.05"

def _format_rgba(self, color: Tuple[float, float, float, float]) -> str:
    """Format color tuple as RGBA string."""
    return f"{color[0]:.3f} {color[1]:.3f} {color[2]:.3f} {color[3]:.3f}"
```

#### 1.4 Update Metadata Extractor

**File**: `src/mujoco_mcp/scene_gen/metadata_extractor.py`

Add method to compute bounding boxes for parametric primitives:

```python
def get_metadata_with_dimensions(
    self, 
    asset_id: str, 
    dimensions: Optional[Dict[str, float]] = None
) -> Optional[AssetMetadata]:
    """Get metadata with computed bounding box for parametric primitives."""
    
    metadata = self.get_metadata(asset_id)
    if not metadata:
        return None
    
    # If parametric primitive and dimensions provided, compute bounding box
    if asset_id.startswith("primitive:") and dimensions:
        primitive_shape = asset_id.split(":")[1]
        bbox_min, bbox_max = self._compute_primitive_bbox(primitive_shape, dimensions)
        
        # Create new metadata with updated bounding box
        metadata = AssetMetadata(
            asset_id=asset_id,
            asset_type=metadata.asset_type,
            description=metadata.description,
            default_dimensions=dimensions,
            semantic_points=self._compute_primitive_semantic_points(primitive_shape, dimensions),
            bounding_box={"min": bbox_min.tolist(), "max": bbox_max.tolist()},
            xml_template=metadata.xml_template
        )
    
    return metadata

def _compute_primitive_bbox(
    self, 
    primitive_shape: str, 
    dimensions: Dict[str, float]
) -> Tuple[np.ndarray, np.ndarray]:
    """Compute bounding box for primitive shapes."""
    
    if primitive_shape == "box":
        w, d, h = dimensions["width"], dimensions["depth"], dimensions["height"]
        return np.array([-w/2, -d/2, 0]), np.array([w/2, d/2, h])
    
    elif primitive_shape == "sphere":
        r = dimensions["radius"]
        return np.array([-r, -r, 0]), np.array([r, r, 2*r])
    
    elif primitive_shape == "cylinder":
        r, h = dimensions["radius"], dimensions["height"]
        return np.array([-r, -r, 0]), np.array([r, r, h])
    
    elif primitive_shape == "capsule":
        r, l = dimensions["radius"], dimensions["length"]
        h = l + 2*r  # Total height including rounded ends
        return np.array([-r, -r, 0]), np.array([r, r, h])
    
    elif primitive_shape == "ellipsoid":
        rx, ry, rz = dimensions["radius_x"], dimensions["radius_y"], dimensions["radius_z"]
        return np.array([-rx, -ry, 0]), np.array([rx, ry, 2*rz])
    
    else:
        return np.array([-0.05, -0.05, 0]), np.array([0.05, 0.05, 0.1])

def _compute_primitive_semantic_points(
    self, 
    primitive_shape: str, 
    dimensions: Dict[str, float]
) -> Dict[str, List[float]]:
    """Compute semantic points for primitive shapes."""
    
    if primitive_shape == "box":
        h = dimensions["height"]
        return {
            "top_surface": [0, 0, h],
            "bottom_surface": [0, 0, 0],
            "center": [0, 0, h/2]
        }
    
    elif primitive_shape in ["sphere", "cylinder", "capsule"]:
        h = dimensions.get("height") or dimensions.get("length", 0.1)
        if primitive_shape == "capsule":
            h += 2 * dimensions["radius"]
        return {
            "top": [0, 0, h],
            "bottom": [0, 0, 0],
            "center": [0, 0, h/2]
        }
    
    else:
        return {"center": [0, 0, 0.05]}
```

#### 1.5 Update LLM System Prompt

**File**: `src/mujoco_mcp/scene_gen/llm_scene_generator.py`

Update `_get_system_prompt()` to include primitive types:

```python
def _get_system_prompt(self) -> str:
    return """You are an expert at generating structured scene descriptions for robotic simulations in MuJoCo.

Your task is to convert natural language descriptions into valid JSON scene descriptions that follow the exact schema below.

## Schema:
```json
{
  "objects": [
    {
      "object_id": "unique_identifier",
      "object_type": "type_from_available_assets_or_primitive",
      "dimensions": {"width": 0.5, "height": 2.0},  // REQUIRED for primitives
      "color": [0.8, 0.2, 0.2, 1.0],  // optional RGBA
      "constraints": [...],
      "orientation": [x, y, z, w]  // optional quaternion
    }
  ],
  "robots": [...],
  "workspace_bounds": [...]
}
```

## Available Primitive Types:

You can create custom-sized objects using these primitive types:

- **primitive:box** - Rectangular box
  - Required dimensions: `width`, `depth`, `height` (in meters)
  - Example: `{"width": 0.3, "depth": 0.3, "height": 0.2}`

- **primitive:sphere** - Spherical object
  - Required dimensions: `radius` (in meters)
  - Example: `{"radius": 0.1}`

- **primitive:cylinder** - Cylindrical object
  - Required dimensions: `radius`, `height` (in meters)
  - Example: `{"radius": 0.02, "height": 2.0}`

- **primitive:capsule** - Rounded cylinder (cylinder with hemispherical caps)
  - Required dimensions: `radius`, `length` (in meters, length excludes caps)
  - Example: `{"radius": 0.03, "length": 0.5}`

- **primitive:ellipsoid** - Ellipsoidal object
  - Required dimensions: `radius_x`, `radius_y`, `radius_z` (in meters)
  - Example: `{"radius_x": 0.1, "radius_y": 0.05, "radius_z": 0.05}`

## Available Predefined Assets:

**Objects:**
- table_standard: Standard work table (0.8m x 1.2m x 0.75m)
- cup_ceramic_small: Small ceramic cup (0.08m diameter, 0.1m height)
- box_small: Small cardboard box (0.1m x 0.1m x 0.1m)
- shelf_small: Small storage shelf with multiple levels

**Robots:**
- franka_panda: 7-DOF robotic arm with gripper

## Available Constraint Types:
[... existing constraint documentation ...]

## Example 1 - Cart Pole with Custom Dimensions:
Input: "Create a cart pole with a 2m long pole"
Output:
```json
{
  "objects": [
    {
      "object_id": "cart",
      "object_type": "primitive:box",
      "dimensions": {
        "width": 0.4,
        "depth": 0.3,
        "height": 0.2
      },
      "color": [0.3, 0.3, 0.3, 1.0],
      "constraints": []
    },
    {
      "object_id": "pole",
      "object_type": "primitive:cylinder",
      "dimensions": {
        "radius": 0.02,
        "height": 2.0
      },
      "color": [0.8, 0.2, 0.2, 1.0],
      "orientation": [0.707, 0, 0, 0.707],
      "constraints": [
        {
          "type": "on_top_of",
          "subject": "pole",
          "reference": "cart",
          "clearance": 0.001
        }
      ]
    }
  ],
  "robots": []
}
```

## Example 2 - Ball on Table:
Input: "Place a red ball on a table"
Output:
```json
{
  "objects": [
    {
      "object_id": "table",
      "object_type": "table_standard",
      "constraints": []
    },
    {
      "object_id": "ball",
      "object_type": "primitive:sphere",
      "dimensions": {
        "radius": 0.1
      },
      "color": [1.0, 0.0, 0.0, 1.0],
      "constraints": [
        {
          "type": "on_top_of",
          "subject": "ball",
          "reference": "table",
          "clearance": 0.001
        }
      ]
    }
  ],
  "robots": []
}
```

## Guidelines:
1. **Use primitive types for custom-sized objects** not in the predefined asset list
2. **Always specify dimensions** when using primitive types
3. Dimensions are in meters
4. Colors are optional RGBA values [0-1]
5. Consider physical realism (a 10kg ball shouldn't be 0.01m radius)
6. Use appropriate primitives for the object (sphere for ball, cylinder for pole, box for cart)
7. Return only valid JSON without markdown formatting

Generate realistic, physically plausible scenes. Respond with valid JSON only."""
```

### Phase 2: Enhanced Metadata & Validation

#### 2.1 Auto-compute Physical Properties

For parametric primitives, automatically compute:
- Mass (based on volume and assumed density)
- Moment of inertia
- Center of mass
- Support surfaces

#### 2.2 Dimension Validation

Add validators to check:
- Dimensions are positive and reasonable (e.g., 0.001m < size < 100m)
- Mass/volume ratios are physically plausible
- Aspect ratios make sense (warn if pole is 100:1 length:radius)

#### 2.3 Enhanced Constraint Solving

Update constraint solver to use parametric bounding boxes:
- Query metadata with dimensions
- Use computed bounding boxes for collision detection
- Adjust semantic points based on actual dimensions

### Phase 3: Expanded Asset Library

#### 3.1 Common Objects

Add predefined assets for common objects:
- Multiple table sizes (small, standard, large)
- Common containers (bucket, bin, drawer, cabinet)
- Kitchen items (plate, bowl, utensil)
- Tools (hammer, wrench, screwdriver)
- Sports equipment (ball variants, racket, bat)

#### 3.2 Robot Models from Menagerie

Integrate more robots:
- UR5e, UR10e (Universal Robots)
- Kinova Gen3
- Shadow Hand
- Mobile bases (TurtleBot, Clearpath)

#### 3.3 Compound Objects

Support multi-part objects:
- Articulated objects (door with hinge)
- Nested objects (drawer in cabinet)
- Attachments (gripper tool changers)

---

## Implementation Priority

### 🔴 High Priority (Core Functionality)
1. ✅ **Schema extension** - Add `dimensions` and `color` fields
2. ✅ **Primitive types** - box, sphere, cylinder in `assets_db.json`
3. ✅ **XML builder updates** - Generate parametric geometry
4. ✅ **LLM prompt** - Document primitive types with examples
5. ✅ **Metadata extractor** - Compute bounding boxes from dimensions

### 🟡 Medium Priority (Robustness)
6. Dimension validation and physical plausibility checks
7. Capsule and ellipsoid primitive support
8. Enhanced constraint solving with parametric metadata
9. Color customization support
10. Semantic point auto-computation

### 🟢 Low Priority (Nice to Have)
11. Expanded static asset database (more tables, objects)
12. Robot model integration from Menagerie
13. Compound/articulated object support
14. Material property specification (friction, density)
15. Texture mapping support

## Spatial Constraint Composition Issues (October 2025)

### Status: Partially Fixed

**Initial fixes applied:**
- ✅ Spatial reasoning now preserves constraint-solved poses (passes `initial_pose` parameter)
- ✅ Enhancement only applies when stable pose exists, otherwise keeps original position

**Remaining Issues** (discovered via test: "Place a table, then line up three cylinders on top"):

### Issue 1: Multi-Constraint Sequential Overwrite ⭐⭐⭐ CRITICAL

**Problem**: When object has multiple constraints (e.g., `[beside A, on_top_of B]`), they execute sequentially and **overwrite** each other instead of composing.

**Current behavior**:
```python
# Orange post has: [beside green_post, on_top_of table]
position = [0, 0, 0]
# Step 1: beside → position = [0, 0.2, 0.951] ✅
# Step 2: on_top_of → position = [0, 0, 1.151] ❌ Lost Y offset!
```

**Root cause**: `_place_constrained_entity` (line 312-324) applies constraints sequentially:
```python
for constraint in constraints:
    position = self._apply_constraint(...)  # Replaces entire position vector
```

Each constraint computes a fresh position from its reference object, ignoring previous constraint results.

**Solution**: Separate constraints by axis they control:
- **Vertical** (`on_top_of`, `inside`) → control Z only
- **Horizontal** (`beside`, `in_front_of`) → control XY only  
- **Full** (`aligned_with_axis`, `within_reach`) → control all axes

Apply horizontal first, then vertical, preserving orthogonal components.

### Issue 2: Dimension Key Mismatch for Cylinders ⭐⭐ IMPORTANT

**Problem**: Cylinders use `"radius"` and `"height"` dimensions, but `_apply_beside` looks for `"depth"`:

```python
# constraint_solver.py:538
ref_half = ref_dims.get("depth", 0.1) / 2.0  # ❌ Cylinder has "radius", not "depth"
```

Falls back to `0.1/2.0 = 0.05`, which coincidentally matches radius but is fragile.

**Solution**: Add `_get_lateral_extent()` helper that handles different primitive types:
- Cylinders: use `2 * radius` for X/Y extent
- Boxes: use `width` (X), `depth` (Y), `height` (Z)

### Issue 3: Z-Position Not Preserved in Horizontal Constraints

**Problem**: `_apply_beside` and `_apply_in_front_of` compute position from `reference_pose.position`, losing Z from previous constraints.

**Solution**: Preserve `current_position[2]` if it's already been set by a vertical constraint.

### Expected Results After Fixes

Test case: "Place a table, then line up three cylinders: 0.4m, 0.8m, 1.2m tall"

| Object | Position | Status |
|--------|----------|--------|
| `table` | `[0, 0, 0]` | ✅ At origin |
| `green_post` (0.4m) | `[0, 0, 0.951]` | ✅ On table: 0.75 + 0.001 + 0.2 |
| `orange_post` (0.8m) | `[0, 0.2, 1.151]` | Currently: `[0, 0, 1.151]` ❌ |
| `blue_post` (1.2m) | `[0, 0.4, 1.551]` | Currently: `[0, 0, 1.551]` ❌ |

All posts should be:
- Above ground (Z > 0)
- On table surface (Z ≈ 0.75 + height/2)
- Lined up in Y direction with 0.2m spacing

---

## Implementation Checklist (Working Copy)

### Phase 1 – Parametric Primitives Enablement
- [x] Extend `ObjectPlacement` schema with `dimensions`/`color` fields, validators, and schema versioning notes
- [x] Introduce `primitive:*` entries in `assets_db.json` with required dimension keys, defaults, and XML templates
- [x] Update metadata extraction to return dimension-aware bounding boxes, semantic points, and expose helper APIs to downstream consumers
- [x] Revise `scene_xml_builder` to format primitive sizes/colors, handle template fallbacks, and log malformed templates clearly
- [x] Refresh LLM system prompt + output parser expectations so primitives and dimensions are documented with canonical examples
- [ ] Regenerate or extend developer docs to explain how to request primitives in natural language prompts

### Phase 2 – Robustness & Physical Fidelity
- [x] **[COMPLETED OCT 2025]** Fix multi-constraint composition - axis-aware constraint application (see Issue #1 above)
- [x] **[COMPLETED OCT 2025]** Add `_get_lateral_extent()` helper for dimension handling across primitive types (see Issue #2 above)
- [x] **[COMPLETED OCT 2025]** Preserve Z in horizontal constraints, XY in vertical constraints (see Issue #3 above)
- [x] **[COMPLETED OCT 2025]** Add LLM prompt Example 3 for multi-constraint patterns (line-up objects)
- [ ] Add dimension sanity checks (min/max bounds, aspect ratio guards) and actionable error messages for the LLM loop
- [ ] Compute derived physics properties (mass, inertia, center of mass) using assumed densities per primitive family
- [ ] Teach constraint solver to consume dimension-aware metadata for placement, clearance, and collision reasoning *(note: current implementation ignores rotated primitive anchors; need orientation-aware semantic points so stacked pendulums land correctly instead of overlapping at 0.1 m)*
- [ ] Apply semantic points in world space (respecting pose orientation) so surfaces such as `table_standard` top (0.73 m + top thickness) are used instead of raw body origins when evaluating `on_top_of`
- [x] Thread RGBA/color preferences through XML generation and ensure defaults remain backward compatible
- [x] Expand automated tests to cover validators, metadata math, prompt→JSON round-trips, and XML parsing for primitives

### Phase 3 – Capability Expansion
- [ ] Publish a richer static asset catalog (tables, storage, kitchenware, tools, sports gear) with consistent naming and metadata
- [ ] Integrate additional robot models (UR series, Kinova, Shadow Hand, mobile bases) into asset DB and prompts
- [ ] Support compound objects and simple articulations (e.g., hinged door, drawer) with scene schema patterns and solver support
- [ ] Add material knobs (density, friction, damping) and document safe defaults for MuJoCo stability
- [ ] Explore texture/visual material assignments for improved realism once core pipeline is stable

### Cross-Cutting Ops & Validation
- [ ] Create regression test scenes targeting common workflows (cart-pole, pendulum, cluttered tabletop) to validate future changes
- [ ] Set up CI hooks to run schema validation + XML generation smoke tests on every change touching scene generation
- [ ] Capture metrics on LLM output quality (dimension completeness, constraint accuracy) to track improvements release-over-release

---

## Testing Strategy

### Unit Tests
- Test each primitive type with various dimensions
- Validate schema with/without dimensions
- Test bounding box computation
- Test XML generation with custom sizes

### Integration Tests
- End-to-end: NL prompt → JSON → XML → MuJoCo load
- Test constraint solving with parametric objects
- Test collision detection with custom sizes

### LLM Tests
- Test prompts that require primitives (cart pole, pendulum, etc.)
- Verify LLM generates correct dimension specifications
- Test dimension plausibility (does LLM understand scale?)

---

## Migration Notes

### Backward Compatibility

The changes should be **backward compatible**:
- Existing `table_standard`, `cup_ceramic_small`, etc. still work
- `dimensions` field is optional for predefined assets
- Existing test scenes continue to work

### Breaking Changes

None expected, but:
- New scenes using primitives won't work with old code
- JSON schema version should be bumped to 2.0

---

## Success Metrics

After implementation, the system should handle:

✅ **User Request**: "Create a cart pole with 2m long pole"
- Generates cart as box: 0.4m × 0.3m × 0.2m
- Generates pole as cylinder: radius 0.02m, height 2.0m
- Correctly orients pole vertically
- Properly constraints pole on cart

✅ **User Request**: "Place three spheres of different sizes on a table"
- Generates table (predefined asset)
- Generates 3 spheres with radius 0.05m, 0.1m, 0.15m
- Applies constraints to place all on table
- Ensures no collisions between spheres

✅ **User Request**: "Create a simple pendulum"
- Generates support structure (box)
- Generates pendulum rod (cylinder)
- Generates pendulum bob (sphere)
- Applies appropriate constraints

---

## Architecture Benefits

This enhancement maintains the clean separation:

```
Natural Language
    ↓
LLM (now understands primitives + dimensions)
    ↓
SceneDescription (extended schema with dimensions)
    ↓
ConstraintSolver (uses parametric bounding boxes)
    ↓
XMLBuilder (generates custom-sized geometry)
    ↓
MuJoCo XML
```

Each component has a clear, focused responsibility. The parametric primitive support flows naturally through the existing pipeline.

---

## Future Directions

### Dynamic Asset Learning
- Allow users to define custom assets that become part of the library
- Learn common patterns (e.g., "desk setup" = table + monitor + keyboard)

### Procedural Generation
- Generate textured surfaces
- Randomize scene variations for RL training
- Auto-generate clutter and realistic environments

### Physics-Aware Generation
- LLM understands stability (don't stack heavy on light)
- Considers reachability for robot tasks
- Suggests optimal object placement for manipulation

### Multi-Modal Input
- Accept images as input (show me a scene, generate XML)
- Accept sketches (draw rough layout, generate precise scene)
- Accept demonstration (record VR interaction, generate repeatable scene)
