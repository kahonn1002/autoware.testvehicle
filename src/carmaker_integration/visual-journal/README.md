# Reusable visual process journal

Open `index.html` in a browser. No build step is required.

## Reuse for another project

Keep these rendering files:

- `index.html`
- `journal.css`
- `journal.js`

Copy `project-data.js` and replace:

1. `project`: title, completion status, date, key facts;
2. `architecture`: final system pipeline nodes;
3. `stages`: chronological implementation steps;
4. each stage's `implementation`: numbered construction/configuration recipe,
   with optional focused source-code snapshots;
5. optional `files`, `config`, `commands`, and `verify` arrays;
6. each stage's `bugs`: symptom, diagnosis, fix, and result;
7. `restart`: lifecycle or recovery state machine.

A stage without a bug can use a top-level `result`. A stage with one or more
bugs gets expandable four-column debug traces automatically. `critical: true`
adds it to the Critical path filter.

The renderer derives the searchable error matrix from every stage's `bugs`
array, so the same information is maintained only once.

Each stage supports this reusable structure:

```javascript
{
  id: "stable-link-name",
  number: "01",
  phase: "FOUNDATION",
  title: "Create the isolated workspace",
  summary: "Short purpose statement.",
  action: "The intended engineering change.",
  implementation: [
    {
      title: "Concrete substep",
      detail: "What was created and why.",
      code: [
        {
          path: "src/package/source.cpp",
          language: "C++",
          startLine: 42,
          content: "const auto value = configure();\napply(value);"
        }
      ]
    }
  ],
  files: ["path/to/config.yaml"],
  config: [{ key: "parameter", value: "configured value" }],
  commands: ["command used to build or verify"],
  verify: ["observable checkpoint before continuing"],
  critical: true,
  bugs: [
    {
      symptom: "Observed log or behavior",
      diagnosis: "Evidence-backed root cause",
      fix: "Change that removed the cause",
      result: "Observed result after the fix"
    }
  ]
}
```

Use `code` for the smallest contiguous excerpt that proves the implementation
step. Keep the text and formatting faithful to the source file, set
`startLine` to the first real source line, and avoid pasting complete files.
The renderer adds an expandable code panel, file metadata, and a numbered
line gutter automatically.

Opening `index.html#stage-stable-link-name` expands and links directly to a
specific stage.

## Suggested journaling method

For each meaningful engineering step, record:

```text
STEP
  ├─ intended change
  ├─ problem observed
  ├─ evidence and root cause
  ├─ fix applied
  └─ verified result
```

Write symptoms as observed evidence, not conclusions. Keep diagnosis separate
from the fix so future readers can reuse the reasoning even when their system
needs a different implementation.
