<#
.SYNOPSIS
  Copia un usuario de SAP Business One 10 (HANA) de una base de compañía a
  otras, usando exclusivamente el Service Layer (OData/REST).

.DESCRIPTION
  Autocontenido para Windows: corre con Windows PowerShell 5.1 (incluido en
  Windows 10/11) o PowerShell 7+. No requiere instalar nada.

  Copia:
    * El usuario (OUSR) con todos sus atributos editables.
    * Sus autorizaciones generales (USR3, coleccion UserPermission).
    * Los grupos de usuario a los que pertenece (OUGR), con sus
      autorizaciones si tu version del Service Layer las expone.
    * La membresia del usuario en esos grupos (USR7), remapeando los ids de
      grupo del origen a los del destino (los grupos se casan por nombre).
    * Opcionalmente, entradas del arbol de permisos (add-ons) que falten en
      el destino (-CopyPermissionTree).

  No copia: contraseñas (se define una nueva al crear el usuario), licencias
  (se administran en el SLD) ni configuraciones personales de UI.

.EXAMPLE
  # Ensayo sin escribir nada:
  .\copy_b1_user.ps1 -Url https://hana01:50000/b1s/v1 -Username manager `
      -SourceDb SBO_PRUEBAS -UserCode jperez `
      -Targets SBO_PROD_MX, SBO_PROD_CO -DryRun -Insecure

.EXAMPLE
  # Copia real, sobrescribiendo permisos si el usuario/grupo ya existe:
  .\copy_b1_user.ps1 -Url https://hana01:50000/b1s/v1 -Username manager `
      -SourceDb SBO_PRUEBAS -UserCode jperez `
      -Targets SBO_PROD_MX, SBO_PROD_CO `
      -UpdateExisting -CopyPermissionTree -Insecure

.NOTES
  Variables de entorno: B1_PASSWORD (login) y B1_NEW_USER_PASSWORD
  (contraseña inicial para usuarios creados en el destino).
#>
param(
    [string]$Url,
    [string]$Username,
    [string]$Password,
    [string]$SourceDb,
    [string[]]$Targets,
    [string]$UserCode,
    [string]$NewUserPassword,
    [switch]$UpdateExisting,
    [switch]$CopyPermissionTree,
    [string[]]$ExcludeFields = @(),
    [switch]$DryRun,
    [switch]$Insecure
)

Set-StrictMode -Off
$ErrorActionPreference = 'Stop'

# ---------------------------------------------------------------------------
# Cliente minimo del Service Layer
# ---------------------------------------------------------------------------

function Enable-InsecureTls {
    if ($PSVersionTable.PSVersion.Major -ge 6) { return }  # PS7 usa -SkipCertificateCheck
    # Windows PowerShell 5.1: confiar en cualquier certificado + forzar TLS 1.2
    if (-not ('TrustAllCertsPolicy' -as [type])) {
        Add-Type @"
using System.Net; using System.Security.Cryptography.X509Certificates;
public class TrustAllCertsPolicy : ICertificatePolicy {
    public bool CheckValidationResult(ServicePoint sp, X509Certificate c,
                                      WebRequest r, int e) { return true; }
}
"@
    }
    [System.Net.ServicePointManager]::CertificatePolicy = New-Object TrustAllCertsPolicy
    [System.Net.ServicePointManager]::SecurityProtocol = `
        [System.Net.SecurityProtocolType]::Tls12
}

function Format-SLError {
    param($ErrorRecord, [string]$What)
    $msg = $ErrorRecord.Exception.Message
    if ($ErrorRecord.ErrorDetails -and $ErrorRecord.ErrorDetails.Message) {
        try {
            $err = ($ErrorRecord.ErrorDetails.Message | ConvertFrom-Json).error
            if ($err) {
                $val = if ($err.message -is [string]) { $err.message }
                       else { $err.message.value }
                $msg = "[$($err.code)] $val"
            }
        } catch { }
    }
    return "$What fallo: $msg"
}

function New-SLContext {
    param([string]$BaseUrl, [string]$CompanyDb,
          [string]$User, [string]$Pass)
    $ctx = @{
        BaseUrl   = $BaseUrl.TrimEnd('/')
        CompanyDb = $CompanyDb
        Web       = New-Object Microsoft.PowerShell.Commands.WebRequestSession
    }
    Invoke-SL $ctx 'POST' 'Login' @{
        CompanyDB = $CompanyDb; UserName = $User; Password = $Pass }
    return $ctx
}

function Close-SLContext {
    param($Ctx)
    try { Invoke-SL $Ctx 'POST' 'Logout' | Out-Null } catch { }
}

function Invoke-SL {
    # Unico punto de contacto HTTP con el Service Layer.
    param($Ctx, [string]$Method, [string]$Path,
          $Body = $null, [hashtable]$Headers = @{})
    $uri = if ($Path -match '^https?://') { $Path }
           else { $Ctx.BaseUrl + '/' + $Path.TrimStart('/') }
    $call = @{
        Uri         = $uri
        Method      = $Method
        WebSession  = $Ctx.Web
        ContentType = 'application/json'
        Headers     = $Headers
    }
    if ($null -ne $Body) {
        # BOM-less UTF-8 para caracteres acentuados
        $json = $Body | ConvertTo-Json -Depth 30
        $call.Body = [System.Text.Encoding]::UTF8.GetBytes($json)
    }
    if ($script:B1Insecure -and $PSVersionTable.PSVersion.Major -ge 6) {
        $call.SkipCertificateCheck = $true
    }
    try {
        return Invoke-RestMethod @call
    } catch {
        throw (Format-SLError $_ "$Method $Path (base '$($Ctx.CompanyDb)')")
    }
}

function Get-SLAll {
    # GET con paginacion OData (odata.nextLink / @odata.nextLink)
    param($Ctx, [string]$Path, [hashtable]$Query = $null)
    $target = $Path
    if ($Query) {
        $pairs = foreach ($e in $Query.GetEnumerator()) {
            $e.Key + '=' + [uri]::EscapeDataString([string]$e.Value)
        }
        $target = $Path + '?' + ($pairs -join '&')
    }
    $rows = @()
    $headers = @{ Prefer = 'odata.maxpagesize=500' }
    while ($true) {
        $data = Invoke-SL $Ctx 'GET' $target -Headers $headers
        if ($null -ne $data.value) { $rows += @($data.value) }
        $next = $data.'odata.nextLink'
        if (-not $next) { $next = $data.'@odata.nextLink' }
        if (-not $next) { return $rows }
        $target = $next
    }
}

# ---------------------------------------------------------------------------
# Utilidades sobre entidades OData
# ---------------------------------------------------------------------------

function Format-ODataString { param([string]$Value)
    return "'" + $Value.Replace("'", "''") + "'"
}

function ConvertTo-CleanEntity {
    # Limpia una entidad para POST/PATCH: quita metadatos OData, claves
    # indicadas y valores nulos (recursivamente). Devuelve hashtable ordenada.
    param($Entity, [string[]]$DropKeys = @())
    $out = [ordered]@{}
    $props = if ($Entity -is [System.Collections.IDictionary]) {
        $Entity.GetEnumerator() | ForEach-Object {
            [pscustomobject]@{ Name = $_.Key; Value = $_.Value } }
    } else { $Entity.PSObject.Properties }
    foreach ($p in $props) {
        $k = [string]$p.Name
        if ($k -like 'odata.*' -or $k -like '*@odata*' -or
            $DropKeys -contains $k) { continue }
        $v = $p.Value
        if ($null -eq $v) { continue }
        if ($v -is [System.Array]) {
            $v = @($v | ForEach-Object {
                if ($_ -is [System.Management.Automation.PSCustomObject] -or
                    $_ -is [System.Collections.IDictionary]) {
                    ConvertTo-CleanEntity $_ $DropKeys
                } else { $_ }
            })
        } elseif ($v -is [System.Management.Automation.PSCustomObject]) {
            $v = ConvertTo-CleanEntity $v $DropKeys
        }
        $out[$k] = $v
    }
    return $out
}

function Get-MetadataInfo {
    # Extrae de $metadata la propiedad clave de cada EntityType y el mapeo
    # EntitySet -> EntityType.
    param($Metadata)
    $info = @{ Keys = @{}; Sets = @{} }
    try {
        $doc = if ($Metadata -is [xml]) { $Metadata } else { [xml]$Metadata }
    } catch { return $info }
    foreach ($et in $doc.SelectNodes("//*[local-name()='EntityType']")) {
        $name = $et.GetAttribute('Name')
        $ref = $et.SelectSingleNode(".//*[local-name()='PropertyRef']")
        if ($name -and $ref) { $info.Keys[$name] = $ref.GetAttribute('Name') }
    }
    foreach ($es in $doc.SelectNodes("//*[local-name()='EntitySet']")) {
        $n = $es.GetAttribute('Name'); $t = $es.GetAttribute('EntityType')
        if ($n -and $t) { $info.Sets[$n] = ($t -split '\.')[-1] }
    }
    return $info
}

function Test-IsIntLike { param($Value)
    return ($Value -is [int]) -or ($Value -is [long]) -or ($Value -is [int16])
}

function Find-NameProperty {
    # Primera propiedad de texto cuyo nombre contenga 'name' (para casar
    # grupos entre bases por nombre y no por id numerico).
    param($Entity)
    foreach ($p in $Entity.PSObject.Properties) {
        if ($p.Name -like 'odata.*' -or $p.Name -like '*@odata*') { continue }
        if ($p.Value -is [string] -and $p.Name -match 'name') { return $p.Name }
    }
    return $null
}

function Find-MembershipCollection {
    # Busca en el JSON del usuario la coleccion que representa sus grupos
    # (USR7): lista de objetos con alguna propiedad numerica que mencione
    # 'group'. Devuelve @{Collection=..; Field=..} o $null.
    param($User)
    foreach ($p in $User.PSObject.Properties) {
        $v = $p.Value
        if (-not ($v -is [System.Array] -and $v.Count -gt 0 -and
                  $v[0] -is [System.Management.Automation.PSCustomObject])) {
            continue
        }
        foreach ($f in $v[0].PSObject.Properties) {
            if ($f.Name -match 'group' -and (Test-IsIntLike $f.Value)) {
                return @{ Collection = $p.Name; Field = $f.Name }
            }
        }
    }
    return $null
}

function Find-GroupMemberField {
    # Layout alternativo: los miembros viven dentro del grupo. Devuelve
    # @{Collection=..; Field=..} si el usuario aparece en el grupo, o $null.
    param($Group, $UserInternalKey, [string]$UserCode)
    foreach ($p in $Group.PSObject.Properties) {
        $v = $p.Value
        if (-not ($v -is [System.Array] -and $v.Count -gt 0 -and
                  $v[0] -is [System.Management.Automation.PSCustomObject])) {
            continue
        }
        foreach ($item in $v) {
            foreach ($f in $item.PSObject.Properties) {
                if ($f.Value -eq $UserInternalKey -and $f.Name -match 'user') {
                    return @{ Collection = $p.Name; Field = $f.Name }
                }
                if (($f.Value -is [string]) -and $f.Value -eq $UserCode) {
                    return @{ Collection = $p.Name; Field = $f.Name }
                }
            }
        }
    }
    return $null
}

function Get-PermissionIds {
    # Reune todos los PermissionID referenciados en las entidades dadas.
    param([object[]]$Entities)
    $ids = New-Object 'System.Collections.Generic.HashSet[string]'
    $stack = New-Object System.Collections.Stack
    foreach ($e in $Entities) { if ($null -ne $e) { $stack.Push($e) } }
    while ($stack.Count -gt 0) {
        $node = $stack.Pop()
        if ($node -is [System.Collections.IDictionary]) {
            foreach ($k in @($node.Keys)) {
                if ($k -eq 'PermissionID' -and $node[$k] -is [string]) {
                    [void]$ids.Add($node[$k])
                } elseif ($null -ne $node[$k]) { $stack.Push($node[$k]) }
            }
        } elseif ($node -is [System.Management.Automation.PSCustomObject]) {
            foreach ($p in $node.PSObject.Properties) {
                if ($p.Name -eq 'PermissionID' -and $p.Value -is [string]) {
                    [void]$ids.Add($p.Value)
                } elseif ($null -ne $p.Value) { $stack.Push($p.Value) }
            }
        } elseif ($node -is [System.Collections.IEnumerable] -and
                  $node -isnot [string]) {
            foreach ($i in $node) { if ($null -ne $i) { $stack.Push($i) } }
        }
    }
    return $ids
}

function Remove-MissingPermissionRows {
    # Quita de las colecciones del payload las filas cuyo PermissionID no
    # existe en el destino (evita que el POST/PATCH completo falle).
    param($Payload, $MissingIds)
    if (-not $MissingIds -or $MissingIds.Count -eq 0) { return $Payload }
    foreach ($k in @($Payload.Keys)) {
        $v = $Payload[$k]
        if ($v -is [System.Array] -and $v.Count -gt 0 -and
            $v[0] -is [System.Collections.IDictionary]) {
            $Payload[$k] = @($v | Where-Object {
                -not ($_.Contains('PermissionID') -and
                      $MissingIds -contains $_['PermissionID']) })
        }
    }
    return $Payload
}

# ---------------------------------------------------------------------------
# Logica principal
# ---------------------------------------------------------------------------

function Get-SourceData {
    # Lee del origen: usuario completo, sus grupos y el esquema relevante.
    param($Ctx, [string]$Code)
    $rows = @(Get-SLAll $Ctx 'Users' @{
        '$filter' = "UserCode eq $(Format-ODataString $Code)" })
    if ($rows.Count -eq 0) {
        throw "El usuario '$Code' no existe en la base '$($Ctx.CompanyDb)'."
    }
    $meta = Get-MetadataInfo (Invoke-SL $Ctx 'GET' '$metadata')
    $userType = if ($meta.Sets['Users']) { $meta.Sets['Users'] } else { 'User' }
    $userKeyProp = if ($meta.Keys[$userType]) { $meta.Keys[$userType] }
                   else { 'InternalKey' }
    $groupType = if ($meta.Sets['UserGroups']) { $meta.Sets['UserGroups'] }
                 else { 'UserGroup' }
    $groupKeyProp = $meta.Keys[$groupType]

    $user = Invoke-SL $Ctx 'GET' "Users($($rows[0].$userKeyProp))"

    $membership = @{ Collection = $null; Field = $null; Layout = $null }
    $groups = @()
    $m = Find-MembershipCollection $user
    if ($m) {
        # Registrar el layout aunque UserGroups no se pueda leer, para que la
        # membresia nunca viaje al destino con ids del origen sin remapear.
        $membership.Collection = $m.Collection
        $membership.Field = $m.Field
        $membership.Layout = 'user'
    }

    $allGroups = $null
    try { $allGroups = @(Get-SLAll $Ctx 'UserGroups') }
    catch {
        Write-Host ("  AVISO: no se pudo leer UserGroups en el origen ($_). " +
                    "Se copiara el usuario sin grupos.")
    }
    if ($null -ne $allGroups) {
        if (-not $groupKeyProp -and $allGroups.Count -gt 0) {
            foreach ($p in $allGroups[0].PSObject.Properties) {
                if ($p.Name -match '^(usergroupid|absoluteentry|id|code)$' -and
                    (Test-IsIntLike $p.Value)) { $groupKeyProp = $p.Name; break }
            }
        }
        if ($m) {
            $memberIds = @($user.($m.Collection) |
                           ForEach-Object { [long]$_.($m.Field) })
            $groups = @($allGroups | Where-Object {
                $memberIds -contains [long]$_.$groupKeyProp })
        } else {
            foreach ($grp in $allGroups) {
                $gm = Find-GroupMemberField $grp $user.$userKeyProp $Code
                if ($gm) {
                    $membership.Collection = $gm.Collection
                    $membership.Field = $gm.Field
                    $membership.Layout = 'group'
                    $groups += ,$grp
                }
            }
        }
    }
    return @{
        User = $user; UserKeyProp = $userKeyProp
        Groups = $groups; GroupKeyProp = $groupKeyProp
        Membership = $membership
    }
}

function Sync-PermissionTree {
    # Verifica que los PermissionID existan en el destino; si faltan y se
    # pidio, copia las entradas del arbol de permisos. Devuelve los que
    # sigan faltando.
    param($SrcCtx, $DstCtx, $NeededIds, [bool]$CopyMissing, [bool]$IsDryRun)
    $existing = @(Get-SLAll $DstCtx 'UserPermissionTree' @{
        '$select' = 'PermissionID' } | ForEach-Object { $_.PermissionID })
    $missing = @($NeededIds | Where-Object { $existing -notcontains $_ } |
                 Sort-Object)
    if ($missing.Count -eq 0) { return @() }
    if (-not $CopyMissing) {
        Write-Host ("  AVISO: estos PermissionID no existen en " +
            "'$($DstCtx.CompanyDb)' y sus filas de permiso se omitiran " +
            "(usa -CopyPermissionTree para copiarlos):`n         " +
            ($missing -join ', '))
        return $missing
    }
    $entries = @()
    foreach ($permId in $missing) {
        try {
            $entries += ,(Invoke-SL $SrcCtx 'GET' `
                "UserPermissionTree($(Format-ODataString $permId))")
        } catch {
            Write-Host "  AVISO: no se pudo leer UserPermissionTree '$permId' en el origen: $_"
        }
    }
    # Padres antes que hijos
    $entries = @($entries | Sort-Object { ([string]$_.ParentID).Length })
    $still = New-Object 'System.Collections.Generic.HashSet[string]'
    foreach ($permId in $missing) { [void]$still.Add($permId) }
    foreach ($entry in $entries) {
        $permId = $entry.PermissionID
        if ($IsDryRun) {
            Write-Host "  [dry-run] POST UserPermissionTree $permId"
            [void]$still.Remove($permId); continue
        }
        try {
            Invoke-SL $DstCtx 'POST' 'UserPermissionTree' `
                (ConvertTo-CleanEntity $entry) | Out-Null
            [void]$still.Remove($permId)
            Write-Host "  Arbol de permisos: creado '$permId'"
        } catch {
            Write-Host ("  AVISO: no se pudo crear el permiso '$permId' en " +
                        "'$($DstCtx.CompanyDb)': $_")
        }
    }
    return @($still)
}

function Sync-Group {
    # Crea (o actualiza) un grupo en el destino y devuelve su id ahi.
    param($DstCtx, $Group, [string]$GroupKeyProp, [string]$NameProp,
          [bool]$Update, [string[]]$Exclude, $MissingIds, [bool]$IsDryRun)
    $gname = $Group.$NameProp
    $found = @(Get-SLAll $DstCtx 'UserGroups' @{
        '$filter' = "$NameProp eq $(Format-ODataString $gname)" })
    if ($found.Count -eq 0) {
        # Segundo intento, insensible a mayusculas
        $found = @(Get-SLAll $DstCtx 'UserGroups' | Where-Object {
            [string]$_.$NameProp -eq [string]$gname })
    }
    $drop = @($Exclude) + $GroupKeyProp
    $payload = ConvertTo-CleanEntity $Group $drop
    $payload = Remove-MissingPermissionRows $payload $MissingIds

    if ($found.Count -gt 0) {
        $targetId = $found[0].$GroupKeyProp
        if ($Update) {
            if ($IsDryRun) {
                Write-Host "  [dry-run] PATCH UserGroups($targetId)  '$gname'"
            } else {
                Invoke-SL $DstCtx 'PATCH' "UserGroups($targetId)" $payload `
                    -Headers @{ 'B1S-ReplaceCollectionsOnPatch' = 'true' } | Out-Null
                Write-Host "  Grupo '$gname': actualizado (id destino $targetId)"
            }
        } else {
            Write-Host ("  Grupo '$gname': ya existe (id destino $targetId); " +
                "no se modifica (usa -UpdateExisting para sincronizar sus permisos)")
        }
        return [long]$targetId
    }

    if ($IsDryRun) {
        Write-Host "  [dry-run] POST UserGroups  '$gname'"
        return [long]-1
    }
    $created = Invoke-SL $DstCtx 'POST' 'UserGroups' $payload
    $targetId = $created.$GroupKeyProp
    Write-Host "  Grupo '$gname': creado (id destino $targetId)"
    return [long]$targetId
}

function Sync-User {
    # Crea o actualiza el usuario en el destino, con permisos y membresias
    # remapeadas a los ids de grupo del destino.
    param($DstCtx, $Src, $GroupIdMap, [string]$NewPassword,
          [bool]$Update, [string[]]$Exclude, $MissingIds, [bool]$IsDryRun)
    $user = $Src.User
    $keyProp = $Src.UserKeyProp
    $code = $user.UserCode
    $membership = $Src.Membership

    $drop = @($Exclude) + $keyProp
    $payload = ConvertTo-CleanEntity $user $drop
    $payload = Remove-MissingPermissionRows $payload $MissingIds

    if ($membership.Layout -eq 'user') {
        $mcol = $membership.Collection; $mfield = $membership.Field
        $remapped = @(); $dropped = @()
        foreach ($item in @($payload[$mcol])) {
            if ($null -eq $item) { continue }
            $srcGid = [long]$item[$mfield]
            if ($GroupIdMap.ContainsKey($srcGid)) {
                $ni = [ordered]@{}
                foreach ($k in $item.Keys) { $ni[$k] = $item[$k] }
                $ni[$mfield] = $GroupIdMap[$srcGid]
                $remapped += ,$ni
            } else { $dropped += $srcGid }
        }
        if ($dropped.Count -gt 0) {
            Write-Host ("  AVISO: se omite la membresia a grupos no " +
                        "replicados (ids origen: $($dropped -join ', '))")
        }
        $payload[$mcol] = $remapped
    }

    $found = @(Get-SLAll $DstCtx 'Users' @{
        '$filter' = "UserCode eq $(Format-ODataString $code)" })
    if ($found.Count -gt 0) {
        $targetKey = $found[0].$keyProp
        if (-not $Update) {
            Write-Host ("  Usuario '$code': ya existe; no se modifica " +
                "(usa -UpdateExisting para sobrescribir permisos)")
            return $targetKey
        }
        $payload.Remove('Password')
        if ($IsDryRun) {
            Write-Host "  [dry-run] PATCH Users($targetKey)  '$code'"
        } else {
            Invoke-SL $DstCtx 'PATCH' "Users($targetKey)" $payload `
                -Headers @{ 'B1S-ReplaceCollectionsOnPatch' = 'true' } | Out-Null
            Write-Host ("  Usuario '$code': actualizado (permisos y " +
                        "membresias reemplazados)")
        }
        return $targetKey
    }

    $payload['Password'] = $NewPassword
    if ($IsDryRun) {
        Write-Host "  [dry-run] POST Users  '$code'"
        return -1
    }
    $created = Invoke-SL $DstCtx 'POST' 'Users' $payload
    $targetKey = $created.$keyProp
    Write-Host "  Usuario '$code': creado (clave interna destino $targetKey)"
    return $targetKey
}

function Sync-GroupSideMembership {
    # Si la membresia vive dentro del grupo, agrega al usuario en la
    # coleccion de miembros de cada grupo destino.
    param($DstCtx, $Src, $GroupIdMap, $TargetUserKey, [bool]$IsDryRun)
    if ($Src.Membership.Layout -ne 'group') { return }
    $mcol = $Src.Membership.Collection; $mfield = $Src.Membership.Field
    $keyProp = $Src.GroupKeyProp
    foreach ($grp in $Src.Groups) {
        $srcGid = [long]$grp.$keyProp
        if (-not $GroupIdMap.ContainsKey($srcGid)) { continue }
        $targetGid = $GroupIdMap[$srcGid]
        if ($targetGid -eq -1) { continue }
        $targetGrp = Invoke-SL $DstCtx 'GET' "UserGroups($targetGid)"
        $members = @($targetGrp.$mcol)
        if (@($members | Where-Object { $_.$mfield -eq $TargetUserKey }).Count) {
            continue
        }
        $template = $null
        foreach ($it in @($grp.$mcol)) {
            if ($null -ne $it.$mfield) { $template = $it; break }
        }
        $newItem = if ($template) { ConvertTo-CleanEntity $template }
                   else { [ordered]@{} }
        $newItem[$mfield] = $TargetUserKey
        $members = @($members | ForEach-Object { ConvertTo-CleanEntity $_ })
        $members += ,$newItem
        if ($IsDryRun) {
            Write-Host "  [dry-run] PATCH UserGroups($targetGid): agregar miembro"
        } else {
            Invoke-SL $DstCtx 'PATCH' "UserGroups($targetGid)" `
                @{ $mcol = $members } `
                -Headers @{ 'B1S-ReplaceCollectionsOnPatch' = 'true' } | Out-Null
            Write-Host "  Membresia agregada en el grupo id destino $targetGid"
        }
    }
}

function Read-PlainPassword { param([string]$Prompt)
    $sec = Read-Host -AsSecureString $Prompt
    $bstr = [Runtime.InteropServices.Marshal]::SecureStringToBSTR($sec)
    try { return [Runtime.InteropServices.Marshal]::PtrToStringAuto($bstr) }
    finally { [Runtime.InteropServices.Marshal]::ZeroFreeBSTR($bstr) }
}

function Invoke-Main {
    if (-not $Url) { throw 'Falta el parametro -Url (Service Layer).' }
    if (-not $Username) { throw 'Falta el parametro -Username.' }
    if (-not $SourceDb) { throw 'Falta el parametro -SourceDb.' }
    if (-not $UserCode) { throw 'Falta el parametro -UserCode.' }
    if (-not $Targets -or $Targets.Count -eq 0) {
        throw 'Falta el parametro -Targets (una o mas bases destino).'
    }
    if ($Url -notmatch '/b1s/') { $script:Url = $Url.TrimEnd('/') + '/b1s/v1' }
    $script:B1Insecure = [bool]$Insecure
    if ($Insecure) { Enable-InsecureTls }

    $pass = $Password
    if (-not $pass) { $pass = $env:B1_PASSWORD }
    if (-not $pass) { $pass = Read-PlainPassword "Contrasena de $Username" }

    # ------------------------------------------------------------------ #
    # 1. Leer todo del origen                                             #
    # ------------------------------------------------------------------ #
    Write-Host "Origen: leyendo usuario '$UserCode' de '$SourceDb'..."
    $srcCtx = New-SLContext $Url $SourceDb $Username $pass
    $failures = 0
    try {
        $src = Get-SourceData $srcCtx $UserCode
        $nameProp = if ($src.Groups.Count -gt 0) {
            Find-NameProperty $src.Groups[0] } else { $null }
        $needed = Get-PermissionIds (@($src.User) + @($src.Groups))

        Write-Host "  Usuario  : $($src.User.UserCode) ($($src.User.UserName))"
        Write-Host "  Superuser: $($src.User.Superuser)"
        Write-Host ("  Permisos directos (USR3): " +
                    @($src.User.UserPermission).Count)
        if ($src.Groups.Count -gt 0) {
            $names = @($src.Groups | ForEach-Object { "'$($_.$nameProp)'" })
            Write-Host "  Grupos   : $($names -join ', ')"
        } else {
            Write-Host '  Grupos   : (ninguno)'
        }

        # -------------------------------------------------------------- #
        # 2. Replicar en cada destino                                     #
        # -------------------------------------------------------------- #
        foreach ($targetDb in $Targets) {
            Write-Host ''
            Write-Host "Destino '$targetDb':"
            try {
                $dstCtx = New-SLContext $Url $targetDb $Username $pass
            } catch {
                Write-Host "  ERROR: $_"; $failures++; continue
            }
            try {
                $missingIds = @(Sync-PermissionTree $srcCtx $dstCtx $needed `
                    ([bool]$CopyPermissionTree) ([bool]$DryRun))

                $groupIdMap = @{}
                foreach ($grp in $src.Groups) {
                    $gid = Sync-Group $dstCtx $grp $src.GroupKeyProp $nameProp `
                        ([bool]$UpdateExisting) $ExcludeFields $missingIds `
                        ([bool]$DryRun)
                    $groupIdMap[[long]$grp.($src.GroupKeyProp)] = $gid
                }

                $exists = @(Get-SLAll $dstCtx 'Users' @{
                    '$filter' = "UserCode eq $(Format-ODataString $UserCode)"
                }).Count -gt 0
                $newPwd = $null
                if (-not $exists -and -not $DryRun) {
                    $newPwd = $NewUserPassword
                    if (-not $newPwd) { $newPwd = $env:B1_NEW_USER_PASSWORD }
                    if (-not $newPwd) {
                        $newPwd = Read-PlainPassword `
                            "Contrasena inicial para '$UserCode' en '$targetDb'"
                    }
                }

                $targetUserKey = Sync-User $dstCtx $src $groupIdMap $newPwd `
                    ([bool]$UpdateExisting) $ExcludeFields $missingIds `
                    ([bool]$DryRun)

                Sync-GroupSideMembership $dstCtx $src $groupIdMap `
                    $targetUserKey ([bool]$DryRun)
            } catch {
                Write-Host "  ERROR: $_"; $failures++
            } finally {
                Close-SLContext $dstCtx
            }
        }
    } finally {
        Close-SLContext $srcCtx
    }

    Write-Host ''
    if ($failures -gt 0) {
        Write-Host "Terminado con $failures destino(s) con error; revisa los mensajes."
        exit 1
    }
    $suffix = if ($DryRun) { '  (dry-run: no se escribio nada)' } else { '' }
    Write-Host "Listo: usuario '$UserCode' replicado en $($Targets -join ', ').$suffix"
}

# Ejecutar solo si el script se invoca directamente (no al hacer dot-source
# para pruebas).
if ($MyInvocation.InvocationName -ne '.') { Invoke-Main }
