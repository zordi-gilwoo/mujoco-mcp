# LLM Integration for Structured Scene Generation

This document describes the enhanced LLM integration for generating MuJoCo scenes from natural language descriptions.

## Overview

The structured scene generation system now supports real LLM integration through OpenAI's API, with automatic fallback to symbolic plan generation for reliability.

## Features

- **OpenAI API Integration**: Direct integration with GPT-4, GPT-3.5-turbo, and other OpenAI models
- **Configurable Parameters**: Full control over model selection, temperature, max tokens, etc.
- **Graceful Fallbacks**: Automatic fallback to symbolic plan generation if LLM fails
- **Error Handling**: Robust error handling for API failures, invalid responses, etc.
- **Environment-Based Configuration**: All settings configurable via environment variables

## Configuration

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `STRUCTURED_SCENE_LLM` | `disabled` | Enable LLM integration (`enabled`/`disabled`) |
| `OPENAI_API_KEY` | None | OpenAI API key (required for LLM mode) |
| `OPENAI_MODEL` | `gpt-4` | OpenAI model to use |
| `OPENAI_TEMPERATURE` | `0.7` | Creativity level (0.0-2.0) |
| `OPENAI_MAX_TOKENS` | `2000` | Maximum response length |

### Setup Instructions

1. **Install Dependencies**:
   ```bash
   pip install openai
   ```

2. **Set Environment Variables**:
   ```bash
   export STRUCTURED_SCENE_LLM=enabled
   export OPENAI_API_KEY=your_api_key_here
   export OPENAI_MODEL=gpt-4  # or gpt-3.5-turbo
   ```

3. **Use the API**:
   ```python
   from src.mujoco_mcp.scene_gen import MetadataExtractor, LLMSceneGenerator
   
   extractor = MetadataExtractor()
   generator = LLMSceneGenerator(extractor)
   
   scene = generator.generate_scene_description(
       "Create a robotics lab with workbench and robot arm"
   )
   ```

## Usage Examples

### Basic Usage

```python
# Initialize generator
extractor = MetadataExtractor()
generator = LLMSceneGenerator(extractor)

# Generate scene from natural language
scene = generator.generate_scene_description(
    "Create a manufacturing cell with assembly table and collaborative robot"
)

print(f"Generated: {len(scene.objects)} objects, {len(scene.robots)} robots")
```

### MCP Tool Integration

```json
{
  "tool": "create_structured_scene",
  "arguments": {
    "natural_language": "Create a pick-and-place workspace with parts bins and robot arm",
    "dry_run": true
  }
}
```

### Advanced Configuration

```python
import os

# Configure advanced settings
os.environ.update({
    'STRUCTURED_SCENE_LLM': 'enabled',
    'OPENAI_API_KEY': 'your_key',
    'OPENAI_MODEL': 'gpt-3.5-turbo',
    'OPENAI_TEMPERATURE': '0.3',  # More deterministic
    'OPENAI_MAX_TOKENS': '1500'
})

generator = LLMSceneGenerator(extractor)
```

## LLM Prompt Engineering

The system uses carefully crafted prompts that include:

- **Detailed Schema**: Complete JSON schema with examples
- **Asset Inventory**: Available objects, robots, and constraints
- **Spatial Guidelines**: Rules for realistic spatial relationships
- **Validation Rules**: Requirements for valid scene descriptions

### Example LLM Interaction

**Input**: "Create a kitchen workspace with appliances and robot chef"

**Generated JSON**:
```json
{
  "objects": [
    {
      "object_id": "kitchen_counter",
      "object_type": "table_standard",
      "constraints": []
    },
    {
      "object_id": "mixing_bowl",
      "object_type": "cup_ceramic_small", 
      "constraints": [
        {
          "type": "on_top_of",
          "subject": "mixing_bowl",
          "reference": "kitchen_counter",
          "clearance": 0.002
        }
      ]
    }
  ],
  "robots": [
    {
      "robot_id": "chef_robot",
      "robot_type": "franka_panda",
      "joint_config": "ready",
      "constraints": [
        {
          "type": "in_front_of",
          "subject": "chef_robot", 
          "reference": "kitchen_counter",
          "clearance": 0.3
        }
      ]
    }
  ]
}
```

## Fallback Behavior

The system provides multiple layers of fallback:

1. **LLM Enabled + API Key**: Uses OpenAI API for scene generation
2. **LLM Enabled, No API Key**: Falls back to symbolic plan generation
3. **API Failure**: Falls back to symbolic plan generation
4. **Invalid LLM Response**: Falls back to symbolic plan generation
5. **Symbolic Plan Failure**: Returns canned example scenes

## Error Handling

Common error scenarios and handling:

- **Missing API Key**: Graceful degradation to symbolic plans
- **API Rate Limits**: Automatic fallback with appropriate logging
- **Invalid JSON**: Parse error handling with fallback
- **Network Issues**: Timeout handling with fallback
- **Model Unavailable**: Model fallback or symbolic plan use

## Performance Considerations

- **Latency**: LLM calls add 2-5 seconds vs. instant symbolic plans
- **Cost**: API usage charges apply for LLM mode
- **Reliability**: Symbolic plans provide 100% uptime fallback
- **Quality**: LLM provides more creative and varied scenes

## Testing

Run the demo to test LLM integration:

```bash
python demo_llm_integration.py
```

Test with different configurations:

```bash
# Test symbolic plan mode (default)
python demo_llm_integration.py

# Test LLM mode (requires API key)
export STRUCTURED_SCENE_LLM=enabled
export OPENAI_API_KEY=your_key
python demo_llm_integration.py
```

## Troubleshooting

### Common Issues

1. **"OpenAI package not installed"**
   - Solution: `pip install openai`

2. **"API key not found"**
   - Solution: Set `OPENAI_API_KEY` environment variable

3. **"Invalid JSON response"**
   - The system automatically falls back to symbolic plans
   - Check logs for LLM response details

4. **"Rate limit exceeded"**
   - The system falls back to symbolic plans
   - Consider using `gpt-3.5-turbo` for higher rate limits

### Debug Logging

Enable detailed logging to debug issues:

```python
import logging
logging.basicConfig(level=logging.DEBUG)
```

## Future Enhancements

- Support for additional LLM providers (Anthropic, local models)
- Fine-tuned models for scene generation
- Prompt optimization and caching
- Batch scene generation
- Visual scene validation