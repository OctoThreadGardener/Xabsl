# Xabsl

The code at `decision/Xabsl`

## Template of Xabsl Option

```
/*
 * Briefly describe the logic of the option.
 *
 * Reads: (list all the symbols used directly in this option, NOT including the ones used by the other options it uses)
 * - symbol1
 * - symbol2
 *
 * Writes: (list all the symbols assigned directly in this option, NOT including the ones assigned by the other options it uses)
 * - symbol3
 * - symbol4
 *
 * Behaves: (list all the behaviors used directly in this option, NOT including the ones assigned by the other options it uses)
 * - example_behavior
 * - behavior_nothing
 */

option opt_example_option {
    initial state example_initial_state {   // 0
        decision {
            if (...) {
                goto example_intermediate_state;
            }
            else {
                stay;
            }
        }
        action {
            opt_another_example_option();
        }
    }

    state example_intermediate_state {     // 1
        decision {
            if (...) {
                goto example_target_state;
            }
            else {
                stay;
            }
        }
        action {
            behavior_example();
        }
    }

    target state example_target_state {     // 2
        decision {
            stay;
        }
        action {
            behavior_nothing();
        }
    }
}
```

Notes:
- Mark the serial number after each state.
- Be careful to include ALL branches in the decision part.
- Where it is necessary to make CPP do some computation, be sure to write detailed explanations and pseudocode in comments.
- Use 4 spaces for indentation.

## Template of commit message

```
Change XXX function in opt_example_option.

Adds symbol: xxx (explain it)
Adds behavior: yyy (explain it)
```

Notes:
- Be sure to compile the whole project before committing!
- Single-line commit message is NOT recommended!
- When adding/remove symbol/behavior, remember to update `xabsl_symbol.xabsl` and `xabsl_behavior.xabsl` as well.
