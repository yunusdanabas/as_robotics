# Merge Improvement Suggestions into Cursor Planning Prompt

## Your Task

You will receive multiple documents containing suggestions, recommendations, and information about improving an IMU-based tele-imitation system codebase. Your goal is to **merge, consolidate, and format** these documents into a **single, comprehensive LLM prompt** specifically designed for **Cursor IDE's planning mode**.

## Input Documents

You will analyze documents that may include:
- Performance optimization suggestions
- Code quality improvements
- Architecture recommendations
- Feature additions
- Technology recommendations
- Implementation guidance
- Code examples and fixes

## Output Requirements

Create a **single, well-structured prompt** that:

1. **Consolidates all suggestions** from multiple sources
2. **Removes duplicates** and merges similar recommendations
3. **Organizes by priority** (Critical/High/Medium/Low)
4. **Groups by category** (Performance, Architecture, Features, etc.)
5. **Formats for Cursor planning mode** with clear, actionable tasks
6. **Includes file references** using proper code citation format
7. **Provides context** about the project

## Cursor Planning Mode Format

The output should follow this structure:

```markdown
# Codebase Improvement Plan: IMU Tele-Imitation System

## Project Context
[Brief overview of the project and current state]

## Critical Issues (Fix First)

### Task 1: [Title]
**Priority:** Critical
**Category:** [Performance/Architecture/Bug Fix/etc.]
**Files Affected:** 
- `filepath:startLine:endLine`
- `filepath:startLine:endLine`

**Description:**
[Clear description of the issue and why it's critical]

**Implementation Steps:**
1. [Specific action item]
2. [Specific action item]
3. [Specific action item]

**Code Changes:**
```cpp
// Before
[code example]

// After
[code example]
```

**Expected Impact:**
[What improvement this will achieve]

**Dependencies:**
[Any prerequisites or related tasks]

---

### Task 2: [Title]
...

## High Priority Improvements

### Task 1: [Title]
[Same structure as above]

...

## Medium Priority Enhancements

### Task 1: [Title]
[Same structure as above]

...

## Low Priority / Future Work

### Task 1: [Title]
[Same structure as above]

...

## Implementation Order

**Week 1:**
- [ ] Task X (Critical)
- [ ] Task Y (Critical)

**Week 2:**
- [ ] Task Z (High Priority)
- [ ] Task W (High Priority)

**Week 3+:**
- [ ] Task A (Medium Priority)
- [ ] Task B (Medium Priority)

## Notes
[Any additional context, warnings, or considerations]
```

## Processing Guidelines

### 1. Consolidation Rules

- **Merge similar suggestions**: If multiple documents suggest the same improvement, combine them into one task
- **Resolve conflicts**: If documents contradict each other, note the conflict and recommend the best approach
- **Prioritize**: Use the highest priority mentioned across all documents
- **Combine details**: Merge all relevant details from different sources

### 2. Organization Principles

**Priority Levels:**
- **Critical**: Bugs, crashes, security issues, blocking problems
- **High**: Significant performance issues, major architectural problems
- **Medium**: Code quality, maintainability, moderate performance gains
- **Low**: Nice-to-have features, minor optimizations, future enhancements

**Categories:**
- Performance Optimization
- Code Quality & Architecture
- Bug Fixes
- Feature Additions
- Documentation
- Testing
- Configuration

### 3. Task Formatting

Each task should include:
- **Clear title** (action-oriented)
- **Priority level**
- **Category**
- **File references** (with line numbers when available)
- **Description** (what and why)
- **Implementation steps** (how, in order)
- **Code examples** (before/after when available)
- **Expected impact** (quantify when possible)
- **Dependencies** (related tasks)

### 4. Code Citation Format

Use Cursor's code reference format:
```
`filepath:startLine:endLine`
```

Example:
```
`esp32_imu_master/src/main.cpp:534:871`
```

### 5. Remove Redundancy

- **Duplicate suggestions**: Merge into one task
- **Overlapping improvements**: Combine into comprehensive task
- **Similar fixes**: Group related changes
- **Repeated explanations**: Keep the most complete version

## Quality Checklist

Before finalizing, ensure:

- [ ] All unique suggestions are included
- [ ] No duplicates remain
- [ ] Tasks are properly prioritized
- [ ] File references are accurate
- [ ] Implementation steps are clear and actionable
- [ ] Code examples are included where relevant
- [ ] Dependencies between tasks are noted
- [ ] Output is formatted for Cursor planning mode
- [ ] Document is well-organized and readable
- [ ] Context is sufficient for implementation

## Example Processing

**Input Document 1:**
- "Optimize I2C reads in timer_callback()"
- "Reduce status check frequency"

**Input Document 2:**
- "I2C communication is too slow"
- "Status checks every 10s are still too frequent"

**Merged Output:**
```markdown
### Task: Optimize I2C Communication Performance
**Priority:** High
**Category:** Performance Optimization
**Files Affected:**
- `esp32_imu_master/src/main.cpp:534:871` (timer_callback)
- `esp32_imu_master/src/main.cpp:157:224` (check_bno055_status)

**Description:**
I2C reads in the timer callback are causing performance bottlenecks. Status checks, even at 10s intervals, add unnecessary overhead. Optimize I2C communication patterns to reduce latency.

**Implementation Steps:**
1. Batch I2C reads where possible
2. Increase status check interval to 30s or make it event-driven
3. Cache sensor data between reads
4. Consider parallel I2C transactions if supported

**Code Changes:**
[Include relevant code examples from documents]

**Expected Impact:**
Reduce I2C overhead by ~40%, improve publish rate to target 50+ Hz

**Dependencies:**
None
```

## Output Specifications

- **Length**: Comprehensive but concise (aim for 2000-5000 words)
- **Format**: Markdown with clear sections
- **Tone**: Professional, actionable, implementation-focused
- **Structure**: Hierarchical (Critical → High → Medium → Low)
- **Completeness**: Include all unique suggestions from all input documents

## Special Instructions

1. **Preserve Technical Details**: Keep all specific code examples, file paths, and technical recommendations
2. **Maintain Context**: Include enough project context for someone new to understand
3. **Actionable Tasks**: Every task should be implementable with clear steps
4. **Quantify Impact**: Include performance metrics, time estimates, or improvement percentages when available
5. **Cross-Reference**: Link related tasks and note dependencies
6. **Remove Noise**: Eliminate redundant explanations, filler text, or non-actionable content

## Final Output

Produce a **single, consolidated markdown document** that:
- Can be directly used in Cursor IDE's planning mode
- Contains all unique suggestions from input documents
- Is organized by priority and category
- Includes actionable implementation steps
- References specific files and code locations
- Provides clear context and expected outcomes

---

**Begin processing the input documents and produce the consolidated Cursor planning prompt.**

