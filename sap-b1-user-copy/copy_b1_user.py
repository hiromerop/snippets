#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
copy_b1_user.py — Copia un usuario de SAP Business One 10 (HANA) de una base
de compañía a otras, usando exclusivamente el Service Layer (OData/REST).

Qué copia:
  * El usuario (OUSR) con todos sus atributos editables.
  * Sus autorizaciones generales (USR3, colección UserPermission).
  * Los grupos de usuario a los que pertenece (OUGR), incluyendo las
    autorizaciones del grupo si tu versión del Service Layer las expone.
  * La membresía del usuario en esos grupos (USR7), ya sea que el Service
    Layer la modele dentro del usuario o dentro del grupo.
  * Opcionalmente, entradas personalizadas del árbol de permisos
    (UserPermissionTree, p. ej. permisos de add-ons) que falten en el destino.

Qué NO copia (limitaciones del Service Layer / del producto):
  * La contraseña del usuario (se define una nueva con --new-user-password).
  * La asignación de licencias (se administra en el SLD / License Server).
  * Configuraciones personales de formularios/UI.

Ejemplos:

  # Ver qué haría, sin escribir nada:
  python copy_b1_user.py --url https://hana01:50000/b1s/v1 \
      --username manager --source-db SBO_PRUEBAS \
      --user-code jperez --targets SBO_PROD_MX SBO_PROD_CO --dry-run

  # Copiar de verdad (pide contraseñas si no van en variables de entorno):
  python copy_b1_user.py --url https://hana01:50000/b1s/v1 \
      --username manager --source-db SBO_PRUEBAS \
      --user-code jperez --targets SBO_PROD_MX SBO_PROD_CO \
      --update-existing --copy-permission-tree

Variables de entorno: B1_PASSWORD (login) y B1_NEW_USER_PASSWORD (contraseña
inicial para usuarios creados en el destino).

Requiere: Python 3.8+, requests  (pip install requests)
"""

import argparse
import getpass
import json
import os
import re
import sys
import xml.etree.ElementTree as ET

try:
    import requests
except ImportError:
    sys.exit("Falta el módulo 'requests'. Instálalo con: pip install requests")


# ---------------------------------------------------------------------------
# Cliente mínimo del Service Layer
# ---------------------------------------------------------------------------

class ServiceLayerError(Exception):
    pass


class ServiceLayer:
    """Sesión contra una base de compañía vía Service Layer."""

    def __init__(self, base_url, company_db, username, password,
                 verify=True, timeout=60):
        self.base_url = base_url.rstrip("/")
        self.company_db = company_db
        self.username = username
        self.password = password
        self.timeout = timeout
        self.http = requests.Session()
        self.http.verify = verify
        self.http.headers.update({
            "Content-Type": "application/json",
            "Prefer": "odata.maxpagesize=500",
        })

    def login(self):
        resp = self.http.post(
            self.base_url + "/Login",
            json={"CompanyDB": self.company_db,
                  "UserName": self.username,
                  "Password": self.password},
            timeout=self.timeout)
        self._check(resp, "Login a '%s'" % self.company_db)

    def logout(self):
        try:
            self.http.post(self.base_url + "/Logout", timeout=self.timeout)
        except requests.RequestException:
            pass

    def _check(self, resp, what):
        if resp.status_code < 400:
            return
        detail = ""
        try:
            err = resp.json().get("error", {})
            msg = err.get("message")
            detail = msg.get("value") if isinstance(msg, dict) else str(msg)
            code = err.get("code")
            detail = "[%s] %s" % (code, detail)
        except Exception:
            detail = resp.text[:500]
        raise ServiceLayerError("%s falló (HTTP %d): %s"
                                % (what, resp.status_code, detail))

    def get(self, path, params=None):
        resp = self.http.get(self.base_url + "/" + path.lstrip("/"),
                             params=params, timeout=self.timeout)
        self._check(resp, "GET %s" % path)
        return resp.json()

    def get_raw(self, path):
        resp = self.http.get(self.base_url + "/" + path.lstrip("/"),
                             timeout=self.timeout)
        self._check(resp, "GET %s" % path)
        return resp.text

    def get_all(self, path, params=None):
        """GET con paginación OData (odata.nextLink / @odata.nextLink)."""
        rows, data = [], self.get(path, params)
        while True:
            rows.extend(data.get("value", []))
            nxt = data.get("odata.nextLink") or data.get("@odata.nextLink")
            if not nxt:
                return rows
            data = self.get(nxt)

    def post(self, path, payload):
        resp = self.http.post(self.base_url + "/" + path.lstrip("/"),
                              json=payload, timeout=self.timeout)
        self._check(resp, "POST %s" % path)
        return resp.json() if resp.text else {}

    def patch(self, path, payload, replace_collections=True):
        headers = {}
        if replace_collections:
            # Reemplaza las colecciones hijas en lugar de anexar filas
            headers["B1S-ReplaceCollectionsOnPatch"] = "true"
        resp = self.http.patch(self.base_url + "/" + path.lstrip("/"),
                               json=payload, headers=headers,
                               timeout=self.timeout)
        self._check(resp, "PATCH %s" % path)


# ---------------------------------------------------------------------------
# Utilidades sobre entidades OData
# ---------------------------------------------------------------------------

def odata_quote(value):
    return "'" + str(value).replace("'", "''") + "'"


def sanitize(entity, drop_keys=()):
    """Limpia una entidad para reenviarla en POST/PATCH: quita metadatos
    OData, claves indicadas y valores nulos (recursivamente)."""
    out = {}
    for key, val in entity.items():
        if key.startswith("odata.") or "@odata" in key or key in drop_keys:
            continue
        if val is None:
            continue
        if isinstance(val, list):
            val = [sanitize(v, drop_keys) if isinstance(v, dict) else v
                   for v in val]
        elif isinstance(val, dict):
            val = sanitize(val, drop_keys)
        out[key] = val
    return out


def parse_metadata_keys(metadata_xml):
    """Extrae de $metadata la propiedad clave de cada EntityType y el mapeo
    EntitySet -> EntityType. Devuelve (claves, sets)."""
    keys, sets_map = {}, {}
    try:
        root = ET.fromstring(metadata_xml)
    except ET.ParseError:
        return keys, sets_map
    for elem in root.iter():
        tag = elem.tag.rsplit("}", 1)[-1]
        if tag == "EntityType":
            name = elem.attrib.get("Name")
            refs = [e.attrib.get("Name") for e in elem.iter()
                    if e.tag.rsplit("}", 1)[-1] == "PropertyRef"]
            if name and refs:
                keys[name] = refs[0]
        elif tag == "EntitySet":
            set_name = elem.attrib.get("Name")
            type_name = (elem.attrib.get("EntityType") or "").rsplit(".", 1)[-1]
            if set_name and type_name:
                sets_map[set_name] = type_name
    return keys, sets_map


def find_name_property(entity):
    """Primera propiedad de texto cuyo nombre contenga 'name' (para casar
    grupos entre bases por nombre en lugar de por id numérico)."""
    for key, val in entity.items():
        if "@odata" in key or key.startswith("odata."):
            continue
        if isinstance(val, str) and re.search(r"name", key, re.I):
            return key
    return None


def find_membership_collection(user_json):
    """Busca en el JSON del usuario la colección que representa sus grupos
    (USR7): una lista de objetos con alguna propiedad que mencione 'group'.
    Devuelve (nombre_coleccion, nombre_campo_id_grupo) o (None, None)."""
    for key, val in user_json.items():
        if not (isinstance(val, list) and val and isinstance(val[0], dict)):
            continue
        for field, fval in val[0].items():
            if re.search(r"group", field, re.I) and isinstance(fval, int):
                return key, field
    return None, None


def find_member_field_in_group(group_json, user_internal_key, user_code):
    """Layout alternativo: los miembros viven dentro del grupo. Busca la
    colección y el campo que referencia al usuario. Devuelve
    (coleccion, campo, plantilla_item) o (None, None, None)."""
    for key, val in group_json.items():
        if not (isinstance(val, list) and val and isinstance(val[0], dict)):
            continue
        for item in val:
            for field, fval in item.items():
                if fval == user_internal_key and re.search(r"user", field, re.I):
                    return key, field, item
                if isinstance(fval, str) and fval == user_code:
                    return key, field, item
    return None, None, None


def collect_permission_ids(*entities):
    """Reúne todos los PermissionID referenciados en las colecciones de
    permisos de las entidades dadas."""
    ids = set()

    def walk(node):
        if isinstance(node, dict):
            for key, val in node.items():
                if key == "PermissionID" and isinstance(val, str):
                    ids.add(val)
                else:
                    walk(val)
        elif isinstance(node, list):
            for item in node:
                walk(item)

    for ent in entities:
        walk(ent)
    return ids


# ---------------------------------------------------------------------------
# Lógica principal
# ---------------------------------------------------------------------------

def fetch_source_data(sl, user_code):
    """Lee del origen: usuario completo, sus grupos y el esquema relevante."""
    rows = sl.get_all("Users", {"$filter": "UserCode eq %s"
                                % odata_quote(user_code)})
    if not rows:
        raise ServiceLayerError("El usuario '%s' no existe en la base '%s'."
                                % (user_code, sl.company_db))
    # El listado puede venir resumido; pedir la entidad completa por clave
    keys, sets_map = parse_metadata_keys(sl.get_raw("$metadata"))
    user_key_prop = keys.get(sets_map.get("Users", "User"), "InternalKey")
    group_key_prop = keys.get(sets_map.get("UserGroups", "UserGroup"))

    user = sl.get("Users(%s)" % rows[0][user_key_prop])

    # Grupos del usuario: primero el layout "membresía dentro del usuario"
    groups, membership = [], {"collection": None, "field": None,
                              "layout": None}
    mcol, mfield = find_membership_collection(user)
    all_groups = None
    try:
        all_groups = sl.get_all("UserGroups")
    except ServiceLayerError as exc:
        print("  AVISO: no se pudo leer UserGroups en el origen (%s). "
              "Se copiará el usuario sin grupos." % exc)

    if mcol:
        # Aunque UserGroups no se haya podido leer, registrar el layout para
        # que la membresía se remapee (o se omita) y nunca viaje con ids
        # del origen.
        membership.update(collection=mcol, field=mfield, layout="user")

    if all_groups is not None:
        if group_key_prop is None and all_groups:
            # Deducir la clave del grupo si $metadata no la dio
            group_key_prop = next(
                (k for k in all_groups[0]
                 if re.match(r"(?i)(usergroupid|absoluteentry|id|code)$", k)
                 and isinstance(all_groups[0][k], int)), None)
        if mcol:
            member_ids = {item.get(mfield) for item in user.get(mcol, [])}
            groups = [g for g in all_groups
                      if g.get(group_key_prop) in member_ids]
        else:
            # Layout alternativo: buscar al usuario dentro de cada grupo
            for grp in all_groups:
                col, field, _ = find_member_field_in_group(
                    grp, user.get(user_key_prop), user_code)
                if col:
                    membership.update(collection=col, field=field,
                                      layout="group")
                    groups.append(grp)

    return {
        "user": user,
        "user_key_prop": user_key_prop,
        "groups": groups,
        "group_key_prop": group_key_prop,
        "membership": membership,
    }


def ensure_permission_tree(src_sl, dst_sl, needed_ids, copy_missing, dry_run):
    """Verifica que los PermissionID existan en el destino; si faltan y se
    pidió, copia las entradas personalizadas del árbol de permisos."""
    existing = {r["PermissionID"] for r in dst_sl.get_all(
        "UserPermissionTree", {"$select": "PermissionID"})}
    missing = sorted(needed_ids - existing)
    if not missing:
        return set()
    if not copy_missing:
        print("  AVISO: estos PermissionID no existen en '%s' y sus filas de "
              "permiso se omitirán (usa --copy-permission-tree para copiarlos):"
              "\n         %s" % (dst_sl.company_db, ", ".join(missing)))
        return set(missing)

    # Copiar padres antes que hijos
    entries = []
    for pid in missing:
        try:
            entries.append(src_sl.get("UserPermissionTree(%s)"
                                      % odata_quote(pid)))
        except ServiceLayerError as exc:
            print("  AVISO: no se pudo leer UserPermissionTree '%s' en el "
                  "origen: %s" % (pid, exc))
    entries.sort(key=lambda e: len(e.get("ParentID") or ""))
    still_missing = set(missing)
    for entry in entries:
        pid = entry["PermissionID"]
        payload = sanitize(entry)
        if dry_run:
            print("  [dry-run] POST UserPermissionTree %s" % pid)
            still_missing.discard(pid)
            continue
        try:
            dst_sl.post("UserPermissionTree", payload)
            still_missing.discard(pid)
            print("  Árbol de permisos: creado '%s'" % pid)
        except ServiceLayerError as exc:
            print("  AVISO: no se pudo crear el permiso '%s' en '%s': %s"
                  % (pid, dst_sl.company_db, exc))
    return still_missing


def strip_missing_permissions(entity, missing_ids):
    """Elimina de las colecciones de la entidad las filas cuyo PermissionID
    no existe en el destino (evita que el POST/PATCH completo falle)."""
    if not missing_ids:
        return entity
    out = {}
    for key, val in entity.items():
        if isinstance(val, list) and val and isinstance(val[0], dict):
            val = [row for row in val
                   if row.get("PermissionID") not in missing_ids]
        out[key] = val
    return out


def ensure_group(dst_sl, group, group_key_prop, name_prop,
                 update_existing, exclude, dry_run):
    """Crea (o actualiza) un grupo en el destino y devuelve su id ahí."""
    gname = group.get(name_prop)
    found = dst_sl.get_all("UserGroups", {
        "$filter": "%s eq %s" % (name_prop, odata_quote(gname))})
    if not found:
        # Fallback insensible a mayúsculas
        candidates = [g for g in dst_sl.get_all("UserGroups")
                      if (g.get(name_prop) or "").lower()
                      == (gname or "").lower()]
        found = candidates

    drop = set(exclude) | {group_key_prop}
    payload = sanitize(group, drop_keys=drop)

    if found:
        target_id = found[0][group_key_prop]
        if update_existing:
            if dry_run:
                print("  [dry-run] PATCH UserGroups(%s)  «%s»"
                      % (target_id, gname))
            else:
                dst_sl.patch("UserGroups(%s)" % target_id, payload)
                print("  Grupo «%s»: actualizado (id destino %s)"
                      % (gname, target_id))
        else:
            print("  Grupo «%s»: ya existe (id destino %s); no se modifica "
                  "(usa --update-existing para sincronizar sus permisos)"
                  % (gname, target_id))
        return target_id

    if dry_run:
        print("  [dry-run] POST UserGroups  «%s»" % gname)
        return -1
    created = dst_sl.post("UserGroups", payload)
    target_id = created.get(group_key_prop)
    print("  Grupo «%s»: creado (id destino %s)" % (gname, target_id))
    return target_id


def upsert_user(dst_sl, src, group_id_map, new_user_password,
                update_existing, exclude, missing_perm_ids, dry_run):
    """Crea o actualiza el usuario en el destino, con permisos y membresías
    remapeadas a los ids de grupo del destino."""
    user = src["user"]
    key_prop = src["user_key_prop"]
    user_code = user["UserCode"]
    membership = src["membership"]

    drop = set(exclude) | {key_prop}
    payload = sanitize(user, drop_keys=drop)
    payload = strip_missing_permissions(payload, missing_perm_ids)

    # Remapear ids de grupo si la membresía vive dentro del usuario
    if membership["layout"] == "user":
        mcol, mfield = membership["collection"], membership["field"]
        remapped, dropped = [], []
        for item in payload.get(mcol, []):
            src_gid = item.get(mfield)
            if src_gid in group_id_map:
                new_item = dict(item)
                new_item[mfield] = group_id_map[src_gid]
                remapped.append(new_item)
            else:
                dropped.append(src_gid)
        if dropped:
            print("  AVISO: se omite la membresía a grupos no replicados "
                  "(ids origen: %s)" % dropped)
        payload[mcol] = remapped

    found = dst_sl.get_all("Users", {"$filter": "UserCode eq %s"
                                     % odata_quote(user_code)})
    if found:
        target_key = found[0][key_prop]
        if not update_existing:
            print("  Usuario '%s': ya existe; no se modifica "
                  "(usa --update-existing para sobrescribir permisos)"
                  % user_code)
            return target_key
        payload.pop("Password", None)
        if dry_run:
            print("  [dry-run] PATCH Users(%s)  '%s'" % (target_key, user_code))
        else:
            dst_sl.patch("Users(%s)" % target_key, payload)
            print("  Usuario '%s': actualizado (permisos y membresías "
                  "reemplazados)" % user_code)
        return target_key

    payload["Password"] = new_user_password
    if dry_run:
        print("  [dry-run] POST Users  '%s'" % user_code)
        return -1
    created = dst_sl.post("Users", payload)
    target_key = created.get(key_prop)
    print("  Usuario '%s': creado (clave interna destino %s)"
          % (user_code, target_key))
    return target_key


def ensure_group_side_membership(dst_sl, src, group_id_map, target_user_key,
                                 dry_run):
    """Si la membresía vive dentro del grupo, agrega al usuario en la
    colección de miembros de cada grupo destino."""
    membership = src["membership"]
    if membership["layout"] != "group":
        return
    mcol, mfield = membership["collection"], membership["field"]
    key_prop = src["group_key_prop"]
    for grp in src["groups"]:
        src_gid = grp.get(key_prop)
        target_gid = group_id_map.get(src_gid)
        if target_gid is None or target_gid == -1:
            continue
        target_grp = dst_sl.get("UserGroups(%s)" % target_gid)
        members = list(target_grp.get(mcol) or [])
        if any(m.get(mfield) == target_user_key for m in members):
            continue
        template = next((dict(m) for m in grp.get(mcol, [])
                         if m.get(mfield) is not None), {})
        template[mfield] = target_user_key
        members.append(sanitize(template))
        if dry_run:
            print("  [dry-run] PATCH UserGroups(%s): agregar miembro"
                  % target_gid)
        else:
            dst_sl.patch("UserGroups(%s)" % target_gid, {mcol: members})
            print("  Membresía agregada en el grupo id destino %s"
                  % target_gid)


def main():
    ap = argparse.ArgumentParser(
        description="Copia un usuario de SAP Business One (HANA) con sus "
                    "grupos y autorizaciones a otras bases de compañía, "
                    "vía Service Layer.")
    ap.add_argument("--url", required=True,
                    help="URL del Service Layer, p. ej. "
                         "https://servidor:50000/b1s/v1")
    ap.add_argument("--username", required=True,
                    help="Usuario B1 con permisos de administración "
                         "(p. ej. manager)")
    ap.add_argument("--password",
                    help="Contraseña de login (o variable B1_PASSWORD; si "
                         "falta se pide en la terminal)")
    ap.add_argument("--source-db", required=True,
                    help="Base de compañía origen (CompanyDB)")
    ap.add_argument("--targets", nargs="+", required=True,
                    help="Bases de compañía destino")
    ap.add_argument("--user-code", required=True,
                    help="UserCode del usuario a copiar")
    ap.add_argument("--new-user-password",
                    help="Contraseña inicial si el usuario se crea en el "
                         "destino (o variable B1_NEW_USER_PASSWORD)")
    ap.add_argument("--update-existing", action="store_true",
                    help="Si el usuario/grupo ya existe en el destino, "
                         "sobrescribir sus permisos para igualar el origen")
    ap.add_argument("--copy-permission-tree", action="store_true",
                    help="Copiar entradas del árbol de permisos (add-ons) "
                         "que falten en el destino")
    ap.add_argument("--exclude-fields", nargs="*", default=[],
                    help="Campos a omitir al copiar (p. ej. Branch "
                         "UserBranchAssignment si las sucursales difieren)")
    ap.add_argument("--dry-run", action="store_true",
                    help="Mostrar lo que se haría sin escribir en el destino")
    ap.add_argument("--insecure", action="store_true",
                    help="No validar el certificado TLS del Service Layer "
                         "(común con certificados autofirmados)")
    args = ap.parse_args()

    if "/b1s/" not in args.url:
        args.url = args.url.rstrip("/") + "/b1s/v1"
    if args.insecure:
        import urllib3
        urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

    password = (args.password or os.environ.get("B1_PASSWORD")
                or getpass.getpass("Contraseña de %s: " % args.username))

    # ------------------------------------------------------------------ #
    # 1. Leer todo del origen                                             #
    # ------------------------------------------------------------------ #
    print("Origen: leyendo usuario '%s' de '%s'..."
          % (args.user_code, args.source_db))
    src_sl = ServiceLayer(args.url, args.source_db, args.username, password,
                          verify=not args.insecure)
    src_sl.login()
    try:
        src = fetch_source_data(src_sl, args.user_code)
        name_prop = (find_name_property(src["groups"][0])
                     if src["groups"] else None)
        needed_perm_ids = collect_permission_ids(src["user"], *src["groups"])

        print("  Usuario  : %s (%s)" % (src["user"].get("UserCode"),
                                        src["user"].get("UserName")))
        print("  Superuser: %s" % src["user"].get("Superuser"))
        print("  Permisos directos (USR3): %d"
              % len(src["user"].get("UserPermission") or []))
        if src["groups"]:
            print("  Grupos   : %s"
                  % ", ".join("«%s»" % g.get(name_prop) for g in src["groups"]))
        else:
            print("  Grupos   : (ninguno)")

        # -------------------------------------------------------------- #
        # 2. Replicar en cada destino                                     #
        # -------------------------------------------------------------- #
        failures = 0
        for target_db in args.targets:
            print("\nDestino '%s':" % target_db)
            dst_sl = ServiceLayer(args.url, target_db, args.username,
                                  password, verify=not args.insecure)
            try:
                dst_sl.login()
            except ServiceLayerError as exc:
                print("  ERROR: %s" % exc)
                failures += 1
                continue
            try:
                missing_ids = ensure_permission_tree(
                    src_sl, dst_sl, needed_perm_ids,
                    args.copy_permission_tree, args.dry_run)

                group_id_map = {}
                for grp in src["groups"]:
                    grp_clean = strip_missing_permissions(grp, missing_ids)
                    gid = ensure_group(
                        dst_sl, grp_clean, src["group_key_prop"], name_prop,
                        args.update_existing, args.exclude_fields,
                        args.dry_run)
                    group_id_map[grp.get(src["group_key_prop"])] = gid

                needs_create = not dst_sl.get_all(
                    "Users", {"$filter": "UserCode eq %s"
                              % odata_quote(args.user_code)})
                new_pwd = None
                if needs_create and not args.dry_run:
                    new_pwd = (args.new_user_password
                               or os.environ.get("B1_NEW_USER_PASSWORD")
                               or getpass.getpass(
                                   "Contraseña inicial para '%s' en '%s': "
                                   % (args.user_code, target_db)))

                target_user_key = upsert_user(
                    dst_sl, src, group_id_map, new_pwd,
                    args.update_existing, args.exclude_fields,
                    missing_ids, args.dry_run)

                ensure_group_side_membership(
                    dst_sl, src, group_id_map, target_user_key, args.dry_run)
            except ServiceLayerError as exc:
                print("  ERROR: %s" % exc)
                failures += 1
            finally:
                dst_sl.logout()
    finally:
        src_sl.logout()

    print()
    if failures:
        sys.exit("Terminado con %d destino(s) con error; revisa los mensajes."
                 % failures)
    print("Listo: usuario '%s' replicado en %s."
          % (args.user_code, ", ".join(args.targets))
          + ("  (dry-run: no se escribió nada)" if args.dry_run else ""))


if __name__ == "__main__":
    main()
