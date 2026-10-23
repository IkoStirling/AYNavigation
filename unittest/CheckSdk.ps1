param([Parameter(Mandatory=$true)][string]$Root)
$ErrorActionPreference='Stop'
$navigationCases=@(
    @{query='手绘路径';first='common.navigation.space'},
    @{query='自定义寻路';first='common.navigation.runtime'},
    @{query='避障';first='common.navigation.runtime'},
    @{query='navigationOrder';first='common.navigation.behavior'},
    @{query='RootKinematicMotor';first='common.navigation.entity'}
)
Push-Location -LiteralPath $Root
try {
    foreach($navigationCase in $navigationCases) {
        $navigationRows=@(& pwsh -NoProfile -File tools/ayapi/ayapi.ps1 search $navigationCase.query -Json | ConvertFrom-Json)
        if($LASTEXITCODE -ne 0 -or $navigationRows.Count -eq 0 -or $navigationRows[0].id -ne $navigationCase.first) {
            throw "Unexpected navigation SDK route: $($navigationCase.query)"
        }
        Write-Output "PASS $($navigationCase.query) -> $($navigationCase.first)"
    }
} finally {Pop-Location}
