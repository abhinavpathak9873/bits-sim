# Agent Coding Rules

Apply these rules before editing this workspace.

## Code Quality

- Prefer existing package structure and helper code over new abstractions.
- Name variables and functions after the domain concept they represent. Avoid generic names such as `data`, `result`, `response`, `item`, `obj`, `info`, and bare `status`.
- Keep functions focused. Split code when a single function starts mixing I/O, validation, transformation, publishing, logging, and plotting.
- Delete dead code, unused imports, abandoned branches, and empty placeholder files.
- Do not add wrapper modules, service classes, or factories unless they remove current complexity.

## Comments

- Do not add docstrings or comments for trivial, self-evident code.
- Comments should explain non-obvious reasoning: simulation quirks, coordinate-frame assumptions, benchmark rules, or ROS/Gazebo workarounds.
- Do not add section-divider comments, AI watermarks, or TODO comments that defer required work.

## Errors And Validation

- Catch specific exceptions only when the code can handle them usefully.
- Do not swallow exceptions or use logging as the whole error handler.
- Error messages should name the failing operation and enough context to debug it.
- Validate user-provided or config-provided values at the boundary where they enter the runner, launch tooling, or simulator setup.

## Security And Safety

- Do not commit secrets, tokens, real credentials, or sensitive environment values.
- Do not build shell commands or SQL-style strings from unchecked user input.
- Avoid logging secrets, tokens, and personally identifying fields.

## Dependencies

- Do not add a dependency for simple standard-library work.
- Reuse existing ROS, Python, CMake, and plotting dependencies when they fit.
- Pin production dependencies to concrete versions when adding manifests.

## Tests

- Add tests for edge cases and failure cases when changing shared behavior.
- Avoid tests that only verify the happy path or mock every meaningful integration point.
- Do not leave empty test files or placeholder TODOs.

Before finishing a change, check that a new engineer can read the touched code quickly and understand what it does, why it exists, and how it fits the simulation or benchmark workflow.
