param([Parameter(Mandatory=$true)][string]$Token)

$ErrorActionPreference = 'Stop'
$headers = @{
    Authorization = "Bearer $Token"
    'Accept'      = 'application/vnd.github+json'
    'X-GitHub-Api-Version' = '2022-11-28'
}
$body = @{
    name        = 's3-xiaozhi-esp32s3'
    private     = $false
    description = 'ESP32-S3 Xiaozhi firmware (sanitized)'
    auto_init   = $false
} | ConvertTo-Json -Compress

try {
    Invoke-RestMethod -Uri 'https://api.github.com/user/repos' -Method Post `
        -Headers $headers -Body $body -ErrorAction Stop | Out-Null
    Write-Host '  Repository created.'
} catch {
    $code = $null
    if ($_.Exception.Response) {
        $code = [int]$_.Exception.Response.StatusCode
    }
    if ($code -eq 422) {
        Write-Host '  Repository already exists, skipped.'
    } else {
        Write-Host "  FAILED to create repository (HTTP $code): $($_.Exception.Message)"
        exit 1
    }
}
