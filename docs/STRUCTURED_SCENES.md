# Structured Scene Generation

## Overview

The Structured Scene Generation system enables the creation of complex MuJoCo simulation scenes from high-level descriptions or natural language prompts. It provides a systematic approach to place objects and robots in 3D space while respecting spatial constraints and avoiding collisions.

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Natural       │    │   JSON Scene    │    │   MCP Tool      │
│   Language      │───▶│   Description   │───▶│   Integration   │
│   Prompt        │    │   (Pydantic)    │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │
                                ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Asset         │    │   Constraint    │    │   XML Scene     │
│   Metadata      │───▶│   Solver        │───▶│   Builder       │
│   Extractor     │    │                 │    │                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                │                        │
                                ▼                        ▼
                        ┌─────────────────┐    ┌─────────────────┐
                        │   Solved 3D     │    │   Complete      │
                        │   Poses         │    │   MuJoCo XML    │
                        │                 │    │                 │
                        └─────────────────┘    └─────────────────┘
```

## Core Components

### 1. Scene Schema (Pydantic Models)

Defines the structured representation of scenes:

- **SpatialConstraint**: Spatial relationships between objects
- **ObjectPlacement**: Object instance with constraints
- **RobotConfiguration**: Robot placement and joint configuration
- **SceneDescription**: Complete scene with validation

### 2. Asset Metadata System

Manages geometric and semantic information about assets:

- **Static Database**: Pre-defined assets in `assets_db.json`
- **Dynamic Extraction**: Extract metadata from MuJoCo XML files
- **Fallback Handling**: Conservative defaults when extraction fails

### 3. Constraint Solver

Solves spatial constraints to determine 3D poses:

- **Supported Constraints**: `on_top_of`, `in_front_of`, `beside`, `no_collision`
- **Greedy Algorithm**: Place unconstrained objects first, then constrained
- **Collision Resolution**: AABB-based collision avoidance

### 4. XML Scene Builder

Composes complete MuJoCo XML scenes:

- **Template System**: Uses asset XML templates with pose substitution
- **Scene Structure**: Adds floor, lighting, and proper worldbody hierarchy
- **Validation**: Ensures generated XML is well-formed

### 5. LLM Integration (Stubbed)

Converts natural language to structured descriptions:

- **Environment Gating**: Controlled by `STRUCTURED_SCENE_LLM` env var
- **Canned Examples**: Default fallback responses
- **Extensible**: Framework for future LLM integration

## Usage

### Via MCP Tool

```json
{
  "tool": "create_structured_scene",
  "arguments": {
    "natural_language": "Create a table with a cup on top and a robot arm in front"
  }
}
```

### Via JSON Description

```json
{
  "tool": "create_structured_scene", 
  "arguments": {
    "scene_description_json": "{\"objects\": [...], \"robots\": [...]}",
    "dry_run": true
  }
}
```

## Available Assets

### Objects
- **table_standard**: Standard rectangular table (1.2m × 0.8m × 0.75m)
- **cup_ceramic_small**: Small ceramic cup with handle
- **box_small**: Small test box (0.1m cube)

### Robots
- **franka_panda**: Franka Emika Panda robot arm
  - Joint configs: `ready`, `home`, `tucked`
  - Reach: 0.855m

## Limitations (Phase 1)

### Current Limitations
- **Collision Detection**: Coarse AABB only (no fine-grained contact)
- **Constraint Types**: Limited to basic spatial relationships
- **Robot Kinematics**: No reachability validation
- **Orientation Constraints**: Not yet implemented
- **Multi-table Scenes**: Limited automatic layout

### Planned Improvements (Phase 2+)
- **Advanced Collision**: MuJoCo contact-based collision detection
- **IK Integration**: Reachability validation using forward kinematics
- **Complex Constraints**: `inside`, `aligned_with_axis`, etc.
- **Backtracking Solver**: Handle conflicting constraints intelligently
- **Real LLM Integration**: OpenAI API or local model integration

## Testing

Run the comprehensive test suite:

```bash
cd /path/to/mujoco-mcp
python -m pytest tests/test_structured_scene_generation.py -v
```