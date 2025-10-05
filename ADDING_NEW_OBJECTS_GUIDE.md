# Guide: Adding New Objects to MuJoCo MCP

**How to scale from 3 composites to 1000+ objects automatically**

---

## 🎯 Three Ways to Add Objects

### Option 1: Simple Predefined Objects (Fastest)
**Use for**: Fixed-size objects that don't need custom dimensions

**Steps**: Add to `assets_db.json`

```json
{
  "plate_dinner": {
    "type": "object",
    "description": "Standard dinner plate",
    "default_dimensions": {
      "radius": 0.14,
      "height": 0.02
    },
    "semantic_points": {
      "top_surface": [0, 0, 0.02],
      "bottom_surface": [0, 0, 0],
      "center": [0, 0, 0.01]
    },
    "bounding_box": {
      "min": [-0.14, -0.14, 0],
      "max": [0.14, 0.14, 0.02]
    },
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"cylinder\" size=\"0.14 0.01\" rgba=\"1 1 1 1\"/></body>"
  }
}
```

**That's it!** The `AssetCatalog` will auto-discover it.

---

### Option 2: New Composite Types (Most Powerful)
**Use for**: Multi-part objects with custom dimensions

**Examples to Add**:
- `composite:drawer` - Pull-out drawer (4 walls + bottom + handle)
- `composite:cabinet` - Enclosed storage (6 walls with door)
- `composite:pallet` - Logistics pallet (slats + blocks)
- `composite:crate` - Shipping crate (6 walls, top removable)

**Steps**:

#### Step 1: Add to `assets_db.json`

```json
{
  "composite:drawer": {
    "type": "composite",
    "composite_shape": "drawer",
    "description": "Pull-out drawer with handle",
    "required_dimensions": ["width", "depth", "height"],
    "optional_dimensions": {
      "wall_thickness": 0.01,
      "handle_size": 0.05
    },
    "default_dimensions": {
      "width": 0.4,
      "depth": 0.4,
      "height": 0.15,
      "wall_thickness": 0.01,
      "handle_size": 0.05
    },
    "semantic_points": {
      "inside_bottom": [0, 0, 0.01],
      "handle_center": [0, -0.2, 0.075],
      "bottom_surface": [0, 0, 0],
      "center": [0, 0, 0.075]
    },
    "bounding_box": {
      "min": [-0.2, -0.2, 0],
      "max": [0.2, 0.2, 0.15]
    },
    "xml_template": "<!-- Composite drawer - generated dynamically -->"
  }
}
```

#### Step 2: Add schema validation to `scene_schema.py`

```python
required_keys = {
    "bin": {"width", "depth", "height"},
    "tote": {"width", "depth", "height"},
    "shelf": {"width", "depth", "height"},
    "drawer": {"width", "depth", "height"},  # ADD THIS LINE
    "cabinet": {"width", "depth", "height"}, # ADD THIS LINE
    # ... more composite types
}
```

#### Step 3: Add geom generator to `scene_xml_builder.py`

```python
def _add_composite_object(self, worldbody, object_id, object_type, pose, dimensions, color, metadata):
    """Generate and add composite objects dynamically."""
    composite_shape = object_type.split(":", 1)[1]
    dims = dimensions or (metadata.get_dimensions() if metadata else {})
    
    rgba_values = color if color is not None else (0.6, 0.6, 0.6, 1.0)
    
    body = ET.SubElement(worldbody, "body")
    body.set("name", object_id)
    body.set("pos", f"{pose.position[0]:.6f} {pose.position[1]:.6f} {pose.position[2]:.6f}")
    body.set("quat", f"{pose.orientation[0]:.6f} {pose.orientation[1]:.6f} {pose.orientation[2]:.6f} {pose.orientation[3]:.6f}")
    
    # Route to appropriate generator
    if composite_shape in ["bin", "tote"]:
        self._generate_bin_geoms(body, object_id, dims, rgba_values)
    elif composite_shape == "shelf":
        self._generate_shelf_geoms(body, object_id, dims, rgba_values)
    elif composite_shape == "drawer":
        self._generate_drawer_geoms(body, object_id, dims, rgba_values)  # ADD THIS
    elif composite_shape == "cabinet":
        self._generate_cabinet_geoms(body, object_id, dims, rgba_values)  # ADD THIS
    else:
        logger.warning(f"Unknown composite shape: {composite_shape}")
```

```python
def _generate_drawer_geoms(self, body, object_id, dimensions, rgba):
    """Generate geoms for drawer (4 walls + bottom + handle)."""
    w = dimensions.get("width", 0.4)
    d = dimensions.get("depth", 0.4)
    h = dimensions.get("height", 0.15)
    t = dimensions.get("wall_thickness", 0.01)
    handle_size = dimensions.get("handle_size", 0.05)
    
    rgba_str = self._format_rgba(rgba)
    
    # Bottom + 4 walls (same as bin)
    # ... (reuse _generate_bin_geoms logic)
    
    # Handle (at front)
    handle = ET.SubElement(body, "geom")
    handle.set("name", f"{object_id}_handle")
    handle.set("type", "cylinder")
    handle.set("size", f"{handle_size/2:.6f} {w/2:.6f}")
    handle.set("pos", f"0 {-d/2:.6f} {h/2:.6f}")
    handle.set("euler", "0 1.57 0")  # Horizontal
    handle.set("rgba", rgba_str)
```

#### Step 4: Add bbox/semantic point computation to `metadata_extractor.py`

```python
def _compute_composite_bbox(self, composite_shape, dimensions):
    """Compute bounds for composite geometry."""
    shape = composite_shape.lower()
    
    if shape in ["bin", "tote"]:
        # ... existing code ...
    elif shape == "drawer":
        w = dimensions.get("width", 0.4)
        d = dimensions.get("depth", 0.4)
        h = dimensions.get("height", 0.15)
        handle_size = dimensions.get("handle_size", 0.05)
        # Include handle in bbox
        return np.array([-w/2, -d/2 - handle_size, 0.0]), np.array([w/2, d/2, h])
    # ... more shapes
```

**Done!** The LLM can now use: `"object_type": "composite:drawer"`

---

### Option 3: Import from External Libraries (Most Scalable)
**Use for**: Real-world object meshes, procedural generation

**Example: YCB Object Dataset** (1000+ household objects)

```python
# In asset_discovery.py, add:
def _discover_ycb_objects(self):
    """Import YCB dataset objects."""
    ycb_path = Path(os.getenv("YCB_DATASET_PATH", ""))
    if not ycb_path.exists():
        return
    
    for obj_dir in ycb_path.iterdir():
        if obj_dir.is_dir():
            obj_id = f"ycb:{obj_dir.name}"
            self.predefined_objects[obj_id] = {
                "type": "object",
                "description": f"YCB object: {obj_dir.name}",
                "mesh_path": str(obj_dir / "textured.obj"),
                "aliases": [obj_dir.name.replace("_", " ").title()],
            }
```

---

## 🚀 How the System Scales Automatically

### The Magic of `AssetCatalog`

```python
# When you add objects to assets_db.json...
catalog = AssetCatalog()

# They're automatically:
# 1. Indexed by keywords
print(catalog.keyword_index["drawer"])  # → {'composite:drawer'}

# 2. Searchable semantically
results = catalog.search_assets("storage with handle")
print(results["composites"])  # → ['composite:drawer', ...]

# 3. Included in LLM prompts (only relevant ones!)
prompt = catalog.format_assets_for_prompt(results)
# Only includes drawer, cabinet, etc. (not irrelevant objects)
```

### Zero Prompt Engineering Required

```python
class LLMSceneGenerator:
    def _get_system_prompt(self, user_prompt):
        # OLD WAY (doesn't scale):
        # return """...
        # Available objects: bin, tote, shelf, drawer, cabinet, ...
        # (1000 lines of hardcoded examples)
        # """
        
        # NEW WAY (scales infinitely):
        relevant_assets = self.asset_catalog.search_assets(user_prompt)
        return f"""...
        {self.asset_catalog.format_assets_for_prompt(relevant_assets)}
        ...
        """
```

**Result**: Prompt size stays constant (~2000 tokens) even with 10,000 objects!

---

## 📊 Example: Adding 10 New Objects in 30 Minutes

Let's add common warehouse/kitchen objects:

```json
{
  "composite:pallet": {
    "type": "composite",
    "composite_shape": "pallet",
    "description": "Logistics pallet for shipping",
    "required_dimensions": ["width", "depth", "height"],
    "default_dimensions": {"width": 1.2, "depth": 0.8, "height": 0.15}
  },
  
  "composite:crate": {
    "type": "composite",
    "composite_shape": "crate",
    "description": "Wooden shipping crate",
    "required_dimensions": ["width", "depth", "height"],
    "default_dimensions": {"width": 0.6, "depth": 0.6, "height": 0.6}
  },
  
  "plate_dinner": {
    "type": "object",
    "description": "Dinner plate",
    "default_dimensions": {"radius": 0.14, "height": 0.02},
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"cylinder\" size=\"0.14 0.01\" rgba=\"1 1 1 1\"/></body>"
  },
  
  "bowl_cereal": {
    "type": "object",
    "description": "Cereal bowl",
    "default_dimensions": {"radius": 0.08, "height": 0.06},
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"cylinder\" size=\"0.08 0.03\" rgba=\"1 1 0.9 1\"/></body>"
  },
  
  "mug_coffee": {
    "type": "object",
    "description": "Coffee mug",
    "default_dimensions": {"radius": 0.04, "height": 0.1},
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"cylinder\" size=\"0.04 0.05\" rgba=\"0.2 0.2 0.2 1\"/></body>"
  },
  
  "knife_kitchen": {
    "type": "object",
    "description": "Kitchen knife",
    "default_dimensions": {"length": 0.25, "width": 0.03, "height": 0.015},
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_blade\" type=\"box\" size=\"0.1 0.015 0.005\" pos=\"0.1 0 0\" rgba=\"0.8 0.8 0.8 1\"/><geom name=\"{name}_handle\" type=\"box\" size=\"0.05 0.015 0.01\" pos=\"-0.05 0 0\" rgba=\"0.2 0.2 0.2 1\"/></body>"
  },
  
  "fork_kitchen": {
    "type": "object",
    "description": "Kitchen fork",
    "default_dimensions": {"length": 0.2, "width": 0.03, "height": 0.01},
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"box\" size=\"0.1 0.015 0.005\" rgba=\"0.8 0.8 0.8 1\"/></body>"
  },
  
  "bottle_water": {
    "type": "object",
    "description": "Water bottle",
    "default_dimensions": {"radius": 0.035, "height": 0.25},
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"cylinder\" size=\"0.035 0.125\" rgba=\"0.3 0.7 1.0 0.6\"/></body>"
  },
  
  "can_soda": {
    "type": "object",
    "description": "Soda can",
    "default_dimensions": {"radius": 0.033, "height": 0.123},
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"cylinder\" size=\"0.033 0.0615\" rgba=\"1.0 0.0 0.0 1\"/></body>"
  },
  
  "book_hardcover": {
    "type": "object",
    "description": "Hardcover book",
    "default_dimensions": {"width": 0.15, "depth": 0.23, "height": 0.035},
    "xml_template": "<body name=\"{name}\" pos=\"{pos}\" quat=\"{quat}\"><geom name=\"{name}_geom\" type=\"box\" size=\"0.075 0.115 0.0175\" rgba=\"0.6 0.3 0.1 1\"/></body>"
  }
}
```

**Test it immediately**:

```python
catalog = AssetCatalog()
print(f"Total objects: {len(catalog.predefined_objects)}")  # → 14 (4 old + 10 new)

results = catalog.search_assets("place a dinner plate and fork on the table")
print(results["objects"])  # → ['plate_dinner', 'fork_kitchen', 'table_standard']
```

---

## 🎨 Advanced: Procedural Generation

For truly unlimited objects, generate them programmatically:

```python
class ProceduralAssetGenerator:
    """Generate unlimited objects procedurally."""
    
    def generate_tool_variants(self, base_tool: str, sizes: List[str]):
        """Generate tool size variants."""
        variants = {}
        size_multipliers = {"small": 0.7, "medium": 1.0, "large": 1.3}
        
        for size in sizes:
            mult = size_multipliers[size]
            variants[f"{base_tool}_{size}"] = {
                "type": "object",
                "description": f"{size.title()} {base_tool}",
                "default_dimensions": {
                    "length": 0.2 * mult,
                    "width": 0.03 * mult,
                    "height": 0.015 * mult
                },
                # ... xml_template
            }
        
        return variants

# Generate 100 tool variants automatically
generator = ProceduralAssetGenerator()
tools = ["wrench", "screwdriver", "hammer", "pliers"]
sizes = ["small", "medium", "large"]

for tool in tools:
    variants = generator.generate_tool_variants(tool, sizes)
    # Add to catalog: 4 tools × 3 sizes = 12 objects
```

---

## 📈 Scaling Roadmap

| Objects | Method | Time Required |
|---------|--------|---------------|
| 10 | Manual JSON entries | 30 min |
| 100 | JSON + templates | 2 hours |
| 1,000 | Import YCB dataset | 1 hour |
| 10,000 | Procedural generation | 1 day |
| 100,000+ | CAD file scanning + AI labeling | 1 week |

---

## ✅ Current Status

- ✅ 5 primitives (box, sphere, cylinder, capsule, ellipsoid)
- ✅ 3 composites (bin, tote, shelf)
- ✅ 4 predefined objects (table, cup, box, shelf_small)
- ✅ 11 robots (auto-discovered from menagerie)

**Total: 23 assets** → Easily scale to **10,000+** using techniques above!

---

## 🎯 Next Steps for Your Team

1. **Short-term** (this week):
   - Add 10 common warehouse objects (pallets, crates, etc.)
   - Add 10 common kitchen objects (plates, utensils, etc.)
   
2. **Medium-term** (this month):
   - Integrate YCB dataset (1000+ objects)
   - Add procedural tool/part generation
   
3. **Long-term** (this quarter):
   - CAD file scanning pipeline
   - AI-powered object labeling
   - User-contributed object library

**Key Insight**: You never modify the LLM prompt! The `AssetCatalog` handles everything automatically through semantic search.
