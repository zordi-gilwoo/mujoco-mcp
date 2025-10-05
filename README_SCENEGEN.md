# Scene Generation Quickstart

This workspace contains a lightweight pipeline that turns natural language prompts into MuJoCo scenes. A prompt is expanded into structured JSON by the LLM, validated against the scene schema, solved for poses, and finally rendered as MuJoCo XML with the local asset database providing geometry metadata.

## Running the generator

```bash
# From the repository root
PYTHONPATH=./src ./text_llm.py "create a cart pole with a 2m long pole"
```

The script prints the structured JSON and the generated MuJoCo XML. Set the `OPENAI_API_KEY` environment variable before running if you want to call the hosted model.

## Example prompts

The prompts below are exercised by the automated tests (see `tests/llm/prompt_scenarios.py`) to guarantee that the downstream pipeline has valid metadata for every asset they reference.

| ID | Prompt |
|----|--------|
| cluttered-workbench | `Create a cluttered workbench with a table, three custom-sized boxes, and a red sphere balanced on the tallest box.` |
| cylinder-row | `Place a table, then line up three cylinders on top of it: a 0.4 m green post, a 0.8 m orange post, and a 1.2 m blue post.` |
| cart-pole | `Build a cart-pole rig with a 0.4 m wide cart and a 1.8 m pole tilted forward 15 degrees.` |
| double-pendulum | `Create a double pendulum with two 1.5 m vertical cylinders connected end to end.` |
| stacked-tower | `Stack three boxes of decreasing size and balance a 0.05 m radius sphere on the smallest box.` |
| storage-corner | `Arrange a workspace with a table, a small storage shelf beside it, and a 0.6 m tall cylinder post on top of the table.` |
| simple-crane | `Build a simple crane using box primitives: a 0.6 m cubic base, a 2 m vertical column, and a 1.5 m horizontal boom.` |

Feel free to copy these directly into `text_llm.py` or adapt them for experimentation.

## Automated checks

The prompt scenarios above are mirrored in the test suite to ensure the metadata pipeline stays healthy:

```bash
PYTHONPATH=./src pytest -p no:sugar tests/llm/test_prompt_scenarios.py
```

The test harness uses stubbed JSON responses for each example prompt, pushes them through the constraint solver and XML builder, and verifies that every referenced asset has metadata in `assets_db.json`. If you add new prompts, update both `README_SCENEGEN.md` and `tests/llm/prompt_scenarios.py` so the documentation and automated checks stay in sync.
