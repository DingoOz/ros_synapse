# Google C++ Style Guide

## Introduction
This document outlines the Google C++ Style Guide, which provides rules and recommendations for writing clear, maintainable, and consistent C++ code. The guide aims to improve code readability and reduce complexity across projects.

## Header Files
- **Self-contained Headers**: Header files should be self-contained and end with `.h`. Avoid using non-standard extensions like `.hpp`.
- **Include Guards**: Use `#define` guards with unique names based on the project and path, e.g., `PROJECT_PATH_FILE_H_`.
- **Forward Declarations**: Prefer forward declarations over unnecessary includes to reduce dependencies.
- **Inline Functions**: Define inline functions in headers only if they are small (e.g., 10 lines or fewer).

## Scoping
- **Namespaces**: Use namespaces to organize code. Avoid nested namespaces that match common top-level directories. End namespace blocks with comments, e.g., `}  // namespace mynamespace`.
- **Unnamed Namespaces**: Use unnamed namespaces in `.cc` files for file-local symbols instead of `static`.
- **Nonmember, Static, and Global Functions**: Prefer nonmember functions within a namespace over static member functions when possible.

## Classes
- **Constructors**: Avoid complex initialization in constructors. Do not call virtual functions in constructors or destructors.
- **Implicit Conversions**: Avoid defining implicit conversion functions or single-argument constructors unless explicitly desired.
- **Structs vs. Classes**: Use `struct` for passive data objects with public members; use `class` for objects with invariants or private data.
- **Inheritance**: Prefer composition over inheritance. Limit inheritance to "is-a" relationships. Virtual functions should be explicitly declared `virtual` or `override`.
- **Multiple Inheritance**: Avoid multiple inheritance except for interface classes with no data members.

## Functions
- **Parameter Ordering**: Input parameters come before output parameters. Use references for output parameters.
- **Short Functions**: Prefer small, focused functions (e.g., 40 lines or fewer).
- **Reference Arguments**: Use `const` references for input parameters unless the function modifies the argument.
- **Function Overloading**: Use overloading sparingly and ensure overloaded functions have distinct purposes.

## Google-Specific Conventions
- **Ownership and Smart Pointers**: Use `std::unique_ptr` for single ownership and `std::shared_ptr` only when shared ownership is required.
- **cpplint**: Run the `cpplint` tool to enforce style rules automatically.

## Formatting
- **Line Length**: Limit lines to 80 characters.
- **Indentation**: Use 2 spaces for indentation, not tabs.
- **Function Declarations**: Place return type and function name on the same line. Break parameters consistently if they exceed the line length.
- **Conditionals**: Place spaces around operators in `if`, `for`, and `while` statements. Use braces for clarity, even for single-line blocks.
- **Pointers and References**: Place the `*` or `&` next to the variable name, not the type, e.g., `int* x`, not `int *x`.

## Naming
- **File Names**: Use lowercase with underscores, e.g., `my_file.cc`.
- **Type Names**: Use `CamelCase` for classes, structs, and typedefs, e.g., `MyClass`.
- **Variable Names**: Use `snake_case` for variables, e.g., `my_variable`. Class members end with an underscore, e.g., `my_member_`.
- **Function Names**: Use `CamelCase` for functions, e.g., `MyFunction()`.
- **Constants**: Use `k` followed by `CamelCase`, e.g., `kMyConstant`.
- **Macros**: Use `UPPER_SNAKE_CASE` for macros, but avoid them when possible.

## Comments
- **Comment Style**: Use `//` for single-line comments and `/* */` for multi-line comments sparingly.
- **File Comments**: Start each file with a license or copyright notice, followed by a description of the file’s purpose.
- **Class Comments**: Document the purpose of each class and its invariants.
- **Function Comments**: Use Doxygen-style comments for functions, describing inputs, outputs, and behavior.
- **Implementation Comments**: Explain complex logic or non-obvious code.

## Exceptions to the Rules
- **Existing Code**: Conform to the style of existing codebases for consistency.
- **Performance**: Deviate from rules only when performance constraints are well-documented and justified.

## Other C++ Features
- **C++ Version**: Use features up to C++17 unless otherwise specified by the project.
- **Auto**: Use `auto` for local variables when the type is obvious or complex, e.g., iterators.
- **Lambdas**: Prefer lambdas for short, local function objects. Name lambdas if captured for clarity.
- **Templates**: Use templates for generic code, but avoid excessive complexity.
- **Standard Library**: Prefer standard library containers and algorithms over custom implementations.

## Inclusive Code
- **Terminology**: Use inclusive language, e.g., avoid terms like “master/slave” or “blacklist/whitelist.”
- **Accessibility**: Ensure code and documentation are accessible, e.g., clear variable names for screen readers.

## Deprecated Practices
- **Exceptions**: Avoid using exceptions unless the codebase already relies on them.
- **Macros**: Minimize macro usage; prefer `constexpr` or `inline` functions.
- **Run-Time Type Information (RTTI)**: Avoid `dynamic_cast` and `typeid` unless necessary.

## Conclusion
Adhering to this style guide ensures that C++ code is consistent, readable, and maintainable across teams. When in doubt, prioritize clarity and simplicity.