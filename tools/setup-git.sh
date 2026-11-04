#!/bin/sh
# Configures this clone for Unity: Smart Merge (UnityYAMLMerge) for scenes and prefabs.
# Run once from the repository root:  sh tools/setup-git.sh [path/to/Unity/Editor]
version=$(grep "m_EditorVersion:" ProjectSettings/ProjectVersion.txt | sed 's/.*: *//' | tr -d '\r')
editor="$1"
if [ -z "$editor" ]; then
  for c in "/c/Program Files/Unity/Hub/Editor/$version/Editor" "/e/Unity/$version/Editor" "/d/Unity/$version/Editor" \
           "/Applications/Unity/Hub/Editor/$version/Unity.app/Contents" "$HOME/Unity/Hub/Editor/$version/Editor"; do
    if [ -f "$c/Data/Tools/UnityYAMLMerge.exe" ] || [ -f "$c/Tools/UnityYAMLMerge" ] || [ -f "$c/Data/Tools/UnityYAMLMerge" ]; then
      editor="$c"; break
    fi
  done
fi
for m in "$editor/Data/Tools/UnityYAMLMerge.exe" "$editor/Tools/UnityYAMLMerge" "$editor/Data/Tools/UnityYAMLMerge"; do
  [ -f "$m" ] && merge="$m" && break
done
if [ -z "$merge" ]; then
  echo "UnityYAMLMerge not found for Unity $version. Run: sh tools/setup-git.sh \"/path/to/Unity/Editor\""
  exit 1
fi
git config merge.unityyamlmerge.name "Unity SmartMerge (UnityYamlMerge)"
git config merge.unityyamlmerge.driver "'$merge' merge -h -p --force %O %B %A %A"
git config merge.unityyamlmerge.recursive binary
git config pull.rebase true
echo "Done: Unity Smart Merge configured with $merge"
