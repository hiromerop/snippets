# Copiar usuarios de SAP Business One 10 (HANA) entre bases — vía Service Layer

Utilitario que replica un usuario de una base de compañía de SAP Business One
a una o varias bases destino **usando exclusivamente el Service Layer**
(OData/REST, `https://servidor:50000/b1s/v1`). No usa DI API ni toca la base
HANA directamente. No es una aplicación web: es una herramienta de línea de
comandos que consume el API del Service Layer.

Viene en dos versiones equivalentes (misma lógica, mismos parámetros):

| Archivo | Para quién |
|---|---|
| **`copy_b1_user.ps1`** | **Windows, autocontenido**: corre con Windows PowerShell 5.1 (incluido en Windows 10/11) o PowerShell 7+. No hay que instalar nada. |
| `copy_b1_user.py` | Cualquier plataforma con Python 3.8+ y `requests`. Se puede empaquetar como `.exe` único con [PyInstaller](https://pyinstaller.org) (`pyinstaller --onefile copy_b1_user.py`). |

## Qué copia

| Elemento | Tabla B1 | Cómo |
|---|---|---|
| El usuario y sus atributos | `OUSR` | Entidad `Users` (POST si no existe, PATCH si existe con `--update-existing`) |
| Autorizaciones generales directas del usuario | `USR3` | Colección `UserPermission` dentro de `Users` |
| Grupos de usuario a los que pertenece | `OUGR` | Entidad `UserGroups`: si el grupo no existe en el destino, se crea con sus autorizaciones |
| Membresía del usuario en sus grupos | `USR7` | Se remapean los ids de grupo del origen a los ids que correspondan en el destino (los grupos se casan **por nombre**, porque los ids numéricos difieren entre bases) |
| Entradas personalizadas del árbol de permisos (add-ons) | `OUPT` | Entidad `UserPermissionTree`, solo con `--copy-permission-tree` |

Notas sobre grupos: en B1, al asignar un usuario a un grupo de autorizaciones,
las autorizaciones efectivas también quedan en `USR3` del usuario; este
utilitario copia ambas cosas (permisos efectivos + membresía), de modo que el
usuario queda igual que en el origen y sigue heredando cambios futuros del
grupo.

## Qué NO copia (limitaciones del producto)

- **Contraseñas**: el Service Layer no permite leerlas. Si el usuario se crea
  en el destino, se le asigna la contraseña de `--new-user-password` (o la
  variable `B1_NEW_USER_PASSWORD`, o se pide en la terminal).
- **Licencias**: la asignación de licencias se administra en el SLD / License
  Server, no por base de compañía.
- **Configuraciones personales de formularios/UI**.
- Si tu versión/FP del Service Layer no expone las autorizaciones del grupo
  (`UGR1`), el grupo se crea con lo que el Service Layer devuelva; los permisos
  efectivos del usuario (`USR3`) se copian de todos modos.

## Requisitos

- SAP Business One 10.0 para SAP HANA con Service Layer habilitado
- Un usuario B1 con permisos de administración en la base origen **y** en cada
  base destino (típicamente `manager`); el mismo login se usa para todas
- El `UserCode` copiado no debe chocar con un usuario distinto ya existente
- Versión PowerShell: nada más (Windows 10/11 ya trae PowerShell 5.1)
- Versión Python: Python 3.8+ y `requests` (`pip install requests`)

## Uso en Windows (PowerShell, autocontenido)

Primero un ensayo sin escribir nada:

```powershell
.\copy_b1_user.ps1 -Url https://hana01:50000/b1s/v1 -Username manager `
    -SourceDb SBO_PRUEBAS -UserCode jperez `
    -Targets SBO_PROD_MX, SBO_PROD_CO -DryRun -Insecure
```

Y la copia real:

```powershell
$env:B1_PASSWORD = '********'            # contraseña del manager
$env:B1_NEW_USER_PASSWORD = 'Inicial1!'  # solo si el usuario no existe en el destino

.\copy_b1_user.ps1 -Url https://hana01:50000/b1s/v1 -Username manager `
    -SourceDb SBO_PRUEBAS -UserCode jperez `
    -Targets SBO_PROD_MX, SBO_PROD_CO `
    -UpdateExisting -CopyPermissionTree -Insecure
```

Los parámetros son los mismos de la tabla de opciones de abajo, en notación
PowerShell (`-DryRun`, `-UpdateExisting`, `-ExcludeFields Branch,
UserBranchAssignment`, etc.). Si al ejecutarlo aparece el aviso de política de
ejecución, corre una vez:
`powershell -ExecutionPolicy Bypass -File .\copy_b1_user.ps1 ...`

## Uso con Python (multiplataforma)

Primero un ensayo sin escribir nada:

```bash
python copy_b1_user.py \
  --url https://hana01:50000/b1s/v1 \
  --username manager \
  --source-db SBO_PRUEBAS \
  --user-code jperez \
  --targets SBO_PROD_MX SBO_PROD_CO \
  --dry-run --insecure
```

Y la copia real:

```bash
export B1_PASSWORD='********'            # contraseña del manager
export B1_NEW_USER_PASSWORD='Inicial1!'  # solo si el usuario no existe en el destino

python copy_b1_user.py \
  --url https://hana01:50000/b1s/v1 \
  --username manager \
  --source-db SBO_PRUEBAS \
  --user-code jperez \
  --targets SBO_PROD_MX SBO_PROD_CO \
  --update-existing --copy-permission-tree --insecure
```

### Opciones

| Opción | Descripción |
|---|---|
| `--url` | URL del Service Layer (si omites `/b1s/v1`, se agrega solo) |
| `--username` / `--password` | Credenciales B1 para todas las bases (`B1_PASSWORD` como alternativa) |
| `--source-db` | Base de compañía origen |
| `--targets` | Una o más bases destino |
| `--user-code` | `UserCode` del usuario a copiar |
| `--new-user-password` | Contraseña inicial si el usuario se crea en el destino |
| `--update-existing` | Si el usuario o el grupo ya existen en el destino, **sobrescribe** sus permisos para dejarlos iguales al origen (usa `B1S-ReplaceCollectionsOnPatch`) |
| `--copy-permission-tree` | Copia entradas del árbol de permisos (p. ej. de add-ons) que falten en el destino; sin esta opción, las filas de permisos huérfanas se omiten con aviso |
| `--exclude-fields` | Campos a no copiar, p. ej. `--exclude-fields Branch UserBranchAssignment` si las sucursales difieren entre bases |
| `--dry-run` | Muestra el plan sin escribir nada |
| `--insecure` | No valida el certificado TLS (común con certificados autofirmados del Service Layer) |

## Cómo funciona por dentro

1. Hace `Login` en la base origen, lee el usuario completo
   (`GET /Users(clave)`), que incluye sus permisos (`USR3`) y su membresía a
   grupos (`USR7`), y lee los `UserGroups` a los que pertenece.
2. El utilitario **descubre el esquema en tiempo de ejecución** (lee
   `$metadata` y examina el JSON): los nombres exactos de la colección de
   membresía y de la clave del grupo varían entre versiones/FP del Service
   Layer, así que no están cableados en el código. Soporta tanto el modelado
   "grupos dentro del usuario" como "miembros dentro del grupo".
3. Por cada base destino: verifica/copia el árbol de permisos, crea o
   actualiza los grupos (casándolos por nombre y construyendo el mapa de ids
   origen→destino), y crea o actualiza el usuario con la membresía remapeada.

## Solución de problemas

- **HTTP 401 en Login**: revisa `CompanyDB` (en HANA es sensible a mayúsculas)
  y que el usuario tenga acceso a esa base.
- **Error de certificado TLS**: usa `--insecure` o instala el certificado del
  Service Layer en el sistema.
- **Error al crear el usuario por un campo específico** (p. ej. sucursales,
  departamentos o empleados que no existen en el destino): excluye ese campo
  con `--exclude-fields` y vuelve a intentar.
- **PermissionID inexistente en destino**: son permisos de add-ons; corre con
  `--copy-permission-tree` o instala el add-on en la base destino.
