# Multi-Provider LLM Integration for Structured Scene Generation

This document describes the comprehensive multi-provider LLM integration system for generating MuJoCo scenes from natural language descriptions with support for OpenAI, Claude, and Gemini.

## Overview

The structured scene generation system supports real LLM integration through multiple AI providers, with intelligent fallback systems for maximum reliability:

- **OpenAI GPT Models**: Industry-leading natural language understanding
- **Claude (Anthropic)**: Advanced reasoning and instruction following  
- **Google Gemini**: Fast and cost-effective scene generation
- **Automatic Fallbacks**: Symbolic plan generation and canned examples

## Supported Providers

### OpenAI (GPT Models)
- **Models**: gpt-4, gpt-3.5-turbo, gpt-4-turbo
- **API Key**: `OPENAI_API_KEY` 
- **Key Format**: `sk-...`
- **Strengths**: Best overall quality, excellent instruction following

### Claude (Anthropic)
- **Models**: claude-3-sonnet-20241022, claude-3-haiku-20240307, claude-3-opus-20240229
- **API Key**: `CLAUDE_API_KEY` or `ANTHROPIC_API_KEY`
- **Key Format**: `sk-ant-...`
- **Strengths**: Superior reasoning, detailed spatial understanding

### Google Gemini
- **Models**: gemini-1.5-pro, gemini-1.5-flash, gemini-pro
- **API Key**: `GEMINI_API_KEY` or `GOOGLE_API_KEY`
- **Key Format**: `AIza...`
- **Strengths**: Fast response times, cost-effective

## Quick Setup

### 1. Enable LLM Integration
```bash
export STRUCTURED_SCENE_LLM=enabled
```

### 2. Choose and Configure Provider

#### OpenAI Setup
```bash
export LLM_PROVIDER=openai
export OPENAI_API_KEY=sk-your-openai-key-here
export OPENAI_MODEL=gpt-4  # optional
```

#### Claude Setup
```bash
export LLM_PROVIDER=claude
export CLAUDE_API_KEY=sk-ant-your-claude-key-here
export CLAUDE_MODEL=claude-3-sonnet-20241022  # optional
```

#### Gemini Setup
```bash
export LLM_PROVIDER=gemini
export GEMINI_API_KEY=AIza-your-gemini-key-here
export GEMINI_MODEL=gemini-1.5-pro  # optional
```

### 3. Install Dependencies
```bash
# Install specific provider packages
pip install openai anthropic google-generativeai

# Or install all providers at once
pip install openai anthropic google-generativeai
```

## Configuration Reference

### Environment Variables

| Variable | Default | Description |
|----------|---------|-------------|
| `STRUCTURED_SCENE_LLM` | `disabled` | Enable LLM integration |
| `LLM_PROVIDER` | `openai` | Provider selection (openai/claude/gemini) |

#### OpenAI Configuration
| Variable | Default | Description |
|----------|---------|-------------|
| `OPENAI_API_KEY` | None | OpenAI API key (required) |
| `OPENAI_MODEL` | `gpt-4` | Model selection |
| `OPENAI_TEMPERATURE` | `0.7` | Creativity level (0.0-1.0) |
| `OPENAI_MAX_TOKENS` | `2000` | Maximum response length |

#### Claude Configuration  
| Variable | Default | Description |
|----------|---------|-------------|
| `CLAUDE_API_KEY` | None | Claude API key (required) |
| `ANTHROPIC_API_KEY` | None | Alternative API key variable |
| `CLAUDE_MODEL` | `claude-3-sonnet-20241022` | Model selection |
| `CLAUDE_TEMPERATURE` | `0.7` | Creativity level (0.0-1.0) |
| `CLAUDE_MAX_TOKENS` | `2000` | Maximum response length |

#### Gemini Configuration
| Variable | Default | Description |
|----------|---------|-------------|
| `GEMINI_API_KEY` | None | Gemini API key (required) |
| `GOOGLE_API_KEY` | None | Alternative API key variable |
| `GEMINI_MODEL` | `gemini-1.5-pro` | Model selection |
| `GEMINI_TEMPERATURE` | `0.7` | Creativity level (0.0-1.0) |
| `GEMINI_MAX_TOKENS` | `2000` | Maximum response length |

## Usage Examples

### Python API - Basic Usage
```python
from mujoco_mcp.scene_gen import MetadataExtractor, LLMSceneGenerator

# Initialize with metadata extractor
extractor = MetadataExtractor()
generator = LLMSceneGenerator(extractor)

# Generate scene from natural language (uses configured provider)
scene = generator.generate_scene_description(
    "Create a kitchen workspace with table, cup, and robot arm"
)

print(f"Generated {len(scene.objects)} objects and {len(scene.robots)} robots")
```

### Python API - Dynamic Provider Switching
```python
# Switch providers dynamically
generator.set_provider_config("claude", "sk-ant-your-key", "claude-3-sonnet-20241022")
scene_claude = generator.generate_scene_description("Complex assembly scenario")

generator.set_provider_config("gemini", "AIza-your-key", "gemini-1.5-pro") 
scene_gemini = generator.generate_scene_description("Simple manipulation task")

generator.set_provider_config("openai", "sk-your-key", "gpt-4")
scene_openai = generator.generate_scene_description("Research laboratory setup")
```

### MCP Tool Integration
```json
{
  "tool": "create_structured_scene",
  "arguments": {
    "natural_language": "Create a robotics lab with multiple workstations and collaborative robots"
  }
}
```

### Web UI Integration
1. Open MuJoCo Remote Viewer
2. Select provider from dropdown: "OpenAI (GPT)", "Claude (Anthropic)", or "Gemini (Google)"
3. Enter your API key for the selected provider
4. Click "Save" to configure
5. Use natural language commands in the freestyle interface

## Fallback System

The system provides three-tier fallback architecture:

### Tier 1: Primary LLM
Your configured provider (OpenAI/Claude/Gemini) with full natural language understanding

### Tier 2: Symbolic Plan Generation  
Keyword-based scene generation using pre-defined templates when LLM fails

### Tier 3: Canned Examples
Static scene templates as final fallback ensuring scene generation always succeeds

### Automatic Fallback Triggers
- Missing or invalid API keys
- Network connectivity issues  
- Rate limit exceeded
- Invalid model responses
- JSON parsing failures
- Provider service outages

## Best Practices

### Provider Selection Guidelines
- **Development/Testing**: Use Gemini for speed and cost-effectiveness
- **Production Quality**: Use GPT-4 for best overall results
- **Complex Reasoning**: Use Claude for sophisticated spatial relationships
- **High Volume**: Consider API rate limits and costs

### Prompt Engineering
```python
# Good: Specific and clear
"Create manipulation workspace with standard table, ceramic cup on table center, Franka robot in front"

# Better: Include constraints and context  
"Design pick-and-place scene: work table as base, target cup on left side, robot arm positioned for optimal reach with ready configuration"

# Best: Complete specification
"Create assembly workspace: industrial table (1.2m x 0.8m), input parts container on left, output container on right, Franka Panda robot centered in front with joints in ready position, ensure collision-free paths between containers"
```

## Testing and Validation

### Run Multi-Provider Demo
```bash
# Test all configured providers
python demo_llm_integration.py

# Get configuration help
python demo_llm_integration.py --help
```

### Health Check
```python
from mujoco_mcp.scene_gen import LLMSceneGenerator, MetadataExtractor

extractor = MetadataExtractor()
generator = LLMSceneGenerator(extractor)

# Check current configuration
print(f"Provider: {generator.provider}")
print(f"Model: {generator.model}") 
print(f"LLM Enabled: {generator.llm_enabled}")
print(f"API Key Available: {bool(generator.api_key)}")
```

## Error Handling

### Common Issues

#### Invalid API Key
```
LLM integration enabled but OPENAI_API_KEY not found
```
**Solution**: Set correct environment variable for your provider

#### Model Not Available
```
The model 'claude-4' does not exist
```
**Solution**: Use valid model names for your provider

#### Rate Limiting
```
Rate limit exceeded for requests
```
**Solution**: System automatically falls back to symbolic plans

#### Network Issues
```
Connection timeout to API endpoint
```
**Solution**: Check connectivity; automatic fallback engaged

### Debugging
```python
# Enable debug logging
import logging
logging.basicConfig(level=logging.DEBUG)

# Test with debug info
generator = LLMSceneGenerator(extractor)
scene = generator.generate_scene_description("debug test")
```

## Performance Characteristics

### Response Times (Typical)
- **OpenAI GPT-4**: 3-8 seconds
- **Claude Sonnet**: 2-6 seconds  
- **Gemini Pro**: 1-4 seconds
- **Symbolic Plans**: 100-500ms
- **Canned Examples**: <100ms

### Cost Considerations
- **OpenAI**: Premium pricing, excellent quality
- **Claude**: Moderate pricing, good reasoning
- **Gemini**: Competitive pricing, fast responses
- **Fallbacks**: Free operation

## Security and Deployment

### API Key Security
```bash
# Never commit keys to version control
echo "*.env" >> .gitignore
echo "OPENAI_API_KEY=your-key" > .env

# Use secrets management in production
kubectl create secret generic llm-keys \
  --from-literal=openai-key=sk-your-key \
  --from-literal=claude-key=sk-ant-your-key \
  --from-literal=gemini-key=AIza-your-key
```

### Production Configuration
```yaml
# Kubernetes deployment example
apiVersion: apps/v1
kind: Deployment
spec:
  template:
    spec:
      containers:
      - name: mujoco-mcp
        env:
        - name: STRUCTURED_SCENE_LLM
          value: "enabled"
        - name: LLM_PROVIDER
          value: "openai"
        - name: OPENAI_API_KEY
          valueFrom:
            secretKeyRef:
              name: llm-keys
              key: openai-key
```

## Advanced Features

### Multi-Provider Validation
```python
# Generate scenes with multiple providers for comparison
providers = ['openai', 'claude', 'gemini']
api_keys = {...}  # Your API keys

results = []
for provider in providers:
    generator.set_provider_config(provider, api_keys[provider])
    scene = generator.generate_scene_description(prompt)
    results.append((provider, scene))

# Compare and validate results
for provider, scene in results:
    print(f"{provider}: {len(scene.objects)} objects, {len(scene.robots)} robots")
```

### Custom Prompt Templates
```python
class SpecializedLLMGenerator(LLMSceneGenerator):
    def _get_system_prompt(self):
        return """Specialized prompt for manufacturing scenarios..."""
        
    def _build_user_message(self, prompt):
        return f"Manufacturing context: {prompt}\nRequirements: safety, efficiency, accessibility"
```

## Monitoring and Analytics

### Key Metrics
- Provider success rates
- Average response times
- Fallback trigger frequency  
- Cost per scene generation
- User satisfaction scores

### Logging
```python
import logging

# Configure provider-specific logging
for provider in ['openai', 'claude', 'gemini']:
    logger = logging.getLogger(f"mujoco_mcp.scene_gen.{provider}")
    logger.setLevel(logging.INFO)
```

## Future Roadmap

### Near-term Enhancements
- **Provider Load Balancing**: Automatic distribution across providers
- **Response Caching**: Cache common scene patterns
- **Quality Scoring**: Automatic scene quality assessment
- **Cost Optimization**: Dynamic provider selection based on cost

### Long-term Vision
- **Custom Fine-tuning**: Provider-specific model fine-tuning
- **Multi-modal Input**: Support for images, sketches, voice
- **Collaborative Editing**: Real-time scene refinement
- **Domain Specialization**: Industry-specific scene generation

## Support Resources

### Documentation
- **OpenAI**: https://platform.openai.com/docs
- **Anthropic**: https://docs.anthropic.com/claude/reference  
- **Google AI**: https://ai.google.dev/docs

### Community Support
- **GitHub Issues**: Bug reports and feature requests
- **Discord**: Real-time community help
- **Stack Overflow**: Technical questions with `mujoco-mcp` tag

### API Rate Limits
- **OpenAI**: Varies by tier (TPM/RPM limits)
- **Claude**: 1000 requests/day (free), higher for paid
- **Gemini**: 60 requests/minute (free), scalable quotas

Monitor usage and implement appropriate rate limiting for production deployments.

---

This multi-provider LLM integration provides robust, scalable scene generation with intelligent fallbacks, ensuring reliable operation across different AI providers while maintaining high-quality output.
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