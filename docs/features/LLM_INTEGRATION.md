# LLM Integration for Scene Generation

## Overview

The scene generation system supports multiple LLM providers for creating MuJoCo simulations from natural language descriptions.

---

## Supported Providers

### OpenAI (GPT Models)
- **Models**: gpt-4, gpt-3.5-turbo, gpt-4-turbo
- **API Key**: `OPENAI_API_KEY`
- **Key Format**: `sk-...`

### Claude (Anthropic)
- **Models**: claude-3-sonnet-20241022, claude-3-haiku-20240307
- **API Key**: `CLAUDE_API_KEY` or `ANTHROPIC_API_KEY`
- **Key Format**: `sk-ant-...`

### Google Gemini
- **Models**: gemini-1.5-pro, gemini-1.5-flash
- **API Key**: `GEMINI_API_KEY` or `GOOGLE_API_KEY`
- **Key Format**: `AIza...`

---

## Quick Setup

### 1. Install Provider Package

```bash
# Choose one or more
pip install openai              # For OpenAI
pip install anthropic           # For Claude
pip install google-generativeai # For Gemini
```

### 2. Configure Provider

**OpenAI:**
```bash
export LLM_PROVIDER=openai
export OPENAI_API_KEY=sk-your-key-here
export OPENAI_MODEL=gpt-4  # optional
```

**Claude:**
```bash
export LLM_PROVIDER=claude
export CLAUDE_API_KEY=sk-ant-your-key-here
export CLAUDE_MODEL=claude-3-sonnet-20241022  # optional
```

**Gemini:**
```bash
export LLM_PROVIDER=gemini
export GEMINI_API_KEY=AIza-your-key-here
export GEMINI_MODEL=gemini-1.5-pro  # optional
```

### 3. Enable LLM Integration

```bash
export STRUCTURED_SCENE_LLM=enabled
```

---

## Usage

### Python API

```python
from mujoco_mcp.scene_gen import LLMSceneGenerator, MetadataExtractor

# Initialize
extractor = MetadataExtractor()
generator = LLMSceneGenerator(extractor)

# Generate scene
scene = generator.generate_scene_description(
    "Create a cart pole with a 2m long pole"
)

# Convert to XML
xml = scene.to_xml()
```

### Dynamic Provider Switching

```python
# Switch providers at runtime
generator.set_provider_config("claude", "sk-ant-...", "claude-3-sonnet-20241022")
scene = generator.generate_scene_description("Complex assembly")

generator.set_provider_config("gemini", "AIza-...", "gemini-1.5-pro")
scene = generator.generate_scene_description("Simple task")
```

### WebRTC Viewer Integration

```bash
# Configure via API
curl -X POST "http://localhost:8000/api/config/api-key" \
  -H "Content-Type: application/json" \
  -d '{"provider": "openai", "api_key": "sk-..."}'

# Generate scene
curl -X POST "http://localhost:8000/api/scene/generate" \
  -H "Content-Type: application/json" \
  -d '{"prompt": "Create a double pendulum"}'
```

---

## Configuration Reference

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `STRUCTURED_SCENE_LLM` | `disabled` | Enable LLM integration |
| `LLM_PROVIDER` | `openai` | Provider (openai/claude/gemini) |

### OpenAI Settings

| Variable | Default |
|----------|---------|
| `OPENAI_API_KEY` | None (required) |
| `OPENAI_MODEL` | `gpt-4` |
| `OPENAI_TEMPERATURE` | `0.7` |
| `OPENAI_MAX_TOKENS` | `2000` |

### Claude Settings

| Variable | Default |
|----------|---------|
| `CLAUDE_API_KEY` | None (required) |
| `CLAUDE_MODEL` | `claude-3-sonnet-20241022` |
| `CLAUDE_TEMPERATURE` | `0.7` |
| `CLAUDE_MAX_TOKENS` | `2000` |

### Gemini Settings

| Variable | Default |
|----------|---------|
| `GEMINI_API_KEY` | None (required) |
| `GEMINI_MODEL` | `gemini-1.5-pro` |
| `GEMINI_TEMPERATURE` | `0.7` |
| `GEMINI_MAX_TOKENS` | `2000` |

---

## Fallback System

The system provides automatic fallback when LLM is unavailable:

1. **Primary**: LLM provider (OpenAI/Claude/Gemini)
2. **Fallback**: Symbolic plan generation (keyword-based)
3. **Final Fallback**: Canned example scenes

### Fallback Triggers
- Missing or invalid API keys
- Network connectivity issues
- Rate limit exceeded
- Invalid LLM responses
- Provider service outages

---

## Troubleshooting

### LLM Integration Failed
```
Error: LLM integration enabled but OPENAI_API_KEY not found
```
**Solution**: Set correct environment variable for your provider

### Model Not Available
```
Error: The model 'claude-4' does not exist
```
**Solution**: Use valid model names (see supported models above)

### Rate Limiting
```
Error: Rate limit exceeded
```
**Solution**: System automatically falls back to symbolic plans

### Network Issues
```
Error: Connection timeout
```
**Solution**: Check connectivity; automatic fallback engaged

---

## Performance

### Response Times (Typical)
- **OpenAI GPT-4**: 3-8 seconds
- **Claude Sonnet**: 2-6 seconds
- **Gemini Pro**: 1-4 seconds
- **Symbolic Plans**: <500ms
- **Canned Examples**: <100ms

---

## Example Prompts

```python
# Simple scenes
"Create a pendulum"
"Create a cart pole"

# Custom dimensions
"Create a cart pole with a 2m long pole"
"Place a red ball on a table"

# Multiple objects
"Place a table with three colored balls on top"

# Robot scenes  
"Put a Franka Panda in front of a table with a cup"
```

---

For complete API documentation, see [py_remote_viewer/README.md](../../py_remote_viewer/README.md).

**Status**: Production Ready (v0.1.0)