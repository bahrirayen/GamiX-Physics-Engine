# Working together

Three people work in one Unity project. These rules keep the merges painless and stop the physics library from
breaking under someone else's feet.

## 1. One-time setup (each member)

1. Install **Unity 6000.6.1f1** exactly. A different version rewrites `ProjectSettings` and the packages, and
   every merge conflicts.
2. Clone the repository and open the folder with Unity Hub.
3. Run the git setup script once from the repository root. It configures Unity's YAML merge tool for scenes and
   prefabs:
   - Windows (PowerShell): `./tools/setup-git.ps1`
   - Git Bash / macOS / Linux: `sh tools/setup-git.sh`
4. In Unity, run **GamiX Physics > Set Code Optimization To Release**, then **GamiX Physics > Create Demo Scenes**.

## 2. Who owns what

| Path | Owner | Rule |
|---|---|---|
| `Assets/Simulations/Oubay/` | Oubay Bouzekri | Only the owner edits it. |
| `Assets/Simulations/Rayen/` | Rayen Bahri | Only the owner edits it. |
| `Assets/Simulations/Yassine/` | Yassine Hemdene | Only the owner edits it. |
| `Assets/GamiXPhysics/` | Shared (the library) | Changes through a pull request reviewed by another member (see 4). |
| `ProjectSettings/`, `Packages/` | Shared | Tell the group before changing anything. One person commits it. |
| `Assets/GamiXPhysics/Demos/Scenes/` | Generated | Do not edit by hand. Re-create them with the menu. |

Your scenes, prefabs, materials and scripts all live in **your own folder**. Then two people never edit the same
file, and a merge never needs to resolve a scene conflict.

Each personal folder has its own assembly definition (`GamiX.Sim.<Name>`). A compile error in one simulation does
not stop the others, and the library cannot depend on a simulation by accident.

## 3. Branches

- `main` always compiles and the tests pass. Nobody commits to it directly.
- Your simulation: branch `sim/<name>` (for example `sim/rayen`). Push it as often as you like.
- A library change: a short-lived branch `lib/<topic>` (for example `lib/rope-cloth-attach`).
- At least once a week, bring `main` into your branch so nothing diverges for long:

  ```bash
  git checkout sim/rayen
  git fetch origin
  git merge origin/main
  ```

- When something is finished, open a pull request into `main` on GitHub. Another member checks it and merges it.

## 4. Changing the library

The library is what we all build on, so:

1. Only shared physics goes in the library: something at least two simulations need. Physics that only your
   simulation needs stays in your folder, as a `SolverExtension` (see the README).
2. Make the change general. "Add a way to tie a rope to a cloth particle", not "make my scene work".
3. Add or update a test in `Assets/GamiXPhysics/Tests/EditMode/`.
4. Don't rename or move public types, fields or methods that someone else might use. If you must, say so in
   the group chat and in the pull request.
5. If the change affects the physics or the parameters, update the README or `docs/TECHNICAL.md`.

## 5. Checklist before every pull request

- [ ] Unity compiles with no errors (Console).
- [ ] **Window > General > Test Runner > EditMode > Run All** is green. It includes a test that fails if any
      script uses Unity physics.
- [ ] No `Rigidbody`, `Collider` (except removing the one `CreatePrimitive` adds), `Physics.*`, joints or
      `OnCollision*` / `OnTrigger*` anywhere.
- [ ] Every new asset is committed **with its `.meta` file**. Never commit a `.meta` without its asset, or an
      asset without its `.meta`.
- [ ] Nothing outside your folder changed unless you meant it (`git status`).
- [ ] No large or temporary files: screenshots, recordings, builds.

## 6. Commits

- Small commits with a clear message in English: `Add wind gusts to the flag simulation`,
  `Fix rope tube normals`.
- Pull before you push: `git pull --rebase` on your own branch.
- If a merge conflict appears in a `.unity` or `.prefab` file, the configured UnityYAMLMerge resolves most of it.
  If it doesn't, keep your own version of files in your folder and theirs for files in their folder. Never hand-edit
  scene YAML.
