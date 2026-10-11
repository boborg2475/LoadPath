# LoadPath

A host-neutral C++20 library that decides which joints in a structure of rigid pieces fail,
in what order, and what moves as a result. It is the structural brain for a physics-destruction
game: the host engine (Unreal, Chaos) owns falling, collision and how a piece fragments; LoadPath
owns whether the structure stands and what lets go when it does not.

- Design brief and host contract: [claude_plans/DESIGN.md](claude_plans/DESIGN.md)
- Working state and open items: [claude_plans/CURRENT_STATE.md](claude_plans/CURRENT_STATE.md)
- Build and test: `Scripts/Test.ps1` (Visual Studio 2022, CMake, doctest)
