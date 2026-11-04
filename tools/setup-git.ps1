# Configures this clone for Unity: Smart Merge (UnityYAMLMerge) for scenes and prefabs.
# Run once from the repository root:  ./tools/setup-git.ps1
param(
    [string]$UnityEditor = ""
)

$version = (Get-Content "ProjectSettings/ProjectVersion.txt" | Select-String "m_EditorVersion:").ToString().Split(":")[1].Trim()

if ($UnityEditor -eq "") {
    $candidates = @(
        "C:/Program Files/Unity/Hub/Editor/$version/Editor",
        "E:/Unity/$version/Editor",
        "D:/Unity/$version/Editor",
        "C:/Unity/$version/Editor"
    )
    foreach ($c in $candidates) {
        if (Test-Path "$c/Data/Tools/UnityYAMLMerge.exe") { $UnityEditor = $c; break }
    }
}

$merge = "$UnityEditor/Data/Tools/UnityYAMLMerge.exe"
if (-not (Test-Path $merge)) {
    Write-Host "UnityYAMLMerge.exe not found for Unity $version."
    Write-Host "Run again with the Editor folder, for example:"
    Write-Host "  ./tools/setup-git.ps1 -UnityEditor 'C:/Program Files/Unity/Hub/Editor/$version/Editor'"
    exit 1
}

git config merge.unityyamlmerge.name "Unity SmartMerge (UnityYamlMerge)"
git config merge.unityyamlmerge.driver "'$merge' merge -h -p --force %O %B %A %A"
git config merge.unityyamlmerge.recursive binary
git config pull.rebase true
Write-Host "Done: Unity Smart Merge configured with $merge"
