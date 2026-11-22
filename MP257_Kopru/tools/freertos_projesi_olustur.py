"""PC'de çalışır: STM32CubeMP2 içindeki iki ST örneğini birleştirip M33 FreeRTOS + OpenAMP projesini oluşturur.

    temel:   Applications/FreeRTOS/FreeRTOS_ThreadCreation  (FreeRTOS, TIM6 zaman tabanı, CoproSync, imzalı derleme)
    eklenen: Applications/OpenAMP/OpenAMP_TTY_echo          (OPENAMP klasörü, libmetal/open-amp kaynakları, define'lar, ön derleme)
    ve bu depodaki m33/freertos/App + m33/common (bağlantı olarak; kopya değil)

Yeni proje aynı derinlikte durur (Applications/F7Sup/F7Sup_FreeRTOS), böylece ST'nin göreli yolları
(../../../../../../../../../Drivers ...) değişmeden çalışır. ST örneklerine hiçbir şey yazılmaz.

    python tools\\freertos_projesi_olustur.py --kontrol     yalnız denetler, dosya yazmaz
    python tools\\freertos_projesi_olustur.py               projeyi oluşturur
"""

import argparse
import os
import re
import shutil
import sys
import xml.etree.ElementTree as ET

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CUBE = r"C:\Users\Asus\Documents\GitHub\STM32MP257F_DK\resources\05_STM32CubeMP2\STM32CubeMP2"
APPS = os.path.join("Projects", "STM32MP257F-DK", "Applications")
OLD_NAME = "FreeRTOS_ThreadCreation_CM33_NonSecure"
NEW_NAME = "F7Sup_FreeRTOS_CM33_NonSecure"
IDE = os.path.join("STM32CubeIDE", "CM33", "NonSecure")

# OpenAMP_TTY_echo'da olup ThreadCreation'da olmayan define'lar (iki .cproject karşılaştırıldı)
OPENAMP_DEFINES = ["LINUX_RPROC_MASTER", "METAL_INTERNAL", "METAL_MAX_DEVICE_REGIONS=2", "NO_ATOMIC_64_SUPPORT",
                   "VIRTIO_DEVICE_SUPPORT=1", "VIRTIO_DRIVER_SUPPORT=0", "VQ_RX_EMPTY_NOTIFY=0"]
OPENAMP_INCLUDES = ["../../../../../../../../../Middlewares/Third_Party/OpenAMP/open-amp/lib/include",
                    "../../../../../../../../../Middlewares/Third_Party/OpenAMP/virtual_driver",
                    "../../../../CM33/NonSecure/OPENAMP",
                    "./libmetal/lib/include"]


def fwd(path):
    return path.replace("\\", "/")


def resolve(location_uri, project_dir):
    """PARENT-n-PROJECT_LOC/... biçimini gerçek yola çevirir; sanal klasörde None."""
    m = re.match(r"PARENT-(\d+)-PROJECT_LOC/(.*)", location_uri)
    if not m:
        return None
    base = project_dir
    for _ in range(int(m.group(1))):
        base = os.path.dirname(base)
    return os.path.join(base, m.group(2))


def link_xml(name, ltype, uri=None, location=None):
    inner = "<locationURI>%s</locationURI>" % uri if uri else "<location>%s</location>" % location
    return "\t\t<link>\n\t\t\t<name>%s</name>\n\t\t\t<type>%d</type>\n\t\t\t%s\n\t\t</link>\n" % (name, ltype, inner)


def build_project_xml(rtos_project, amp_project):
    """ThreadCreation .project + TTY_echo'nun OpenAMP bağlantıları + bu deponun kaynakları."""
    text = rtos_project.replace("<name>%s</name>" % OLD_NAME, "<name>%s</name>" % NEW_NAME, 1)
    have = set(re.findall(r"<name>([^<]+)</name>", text))
    added = []
    for m in re.finditer(r"<link>\s*<name>([^<]+)</name>\s*<type>(\d+)</type>\s*<locationURI>([^<]+)</locationURI>\s*</link>",
                         amp_project):
        name, ltype, uri = m.group(1), int(m.group(2)), m.group(3)
        if (name.startswith("Middlewares/OpenAMP") or name.startswith("Application/User/OPENAMP")) and name not in have:
            added.append(link_xml(name, ltype, uri=uri))
    ours = [("F7Sup/App/app.c", "m33/freertos/App/Src/app.c"),
            ("F7Sup/App/app_rpmsg.c", "m33/freertos/App/Src/app_rpmsg.c"),
            ("F7Sup/Common/link_proto.c", "m33/common/link_proto.c"),
            ("F7Sup/Common/supervisor.c", "m33/common/supervisor.c")]
    for name, rel in ours:
        added.append(link_xml(name, 1, location=fwd(os.path.join(REPO, rel))))
    for folder in ("F7Sup", "F7Sup/App", "F7Sup/Common"):
        added.insert(0, link_xml(folder, 2, uri="virtual:/virtual"))
    text = text.replace("\t</linkedResources>", "".join(added) + "\t</linkedResources>", 1)
    return text, len(added)


def build_cproject_xml(rtos_cproject, amp_cproject):
    """Her yapılandırmaya OpenAMP include/define'larını, bizim include'ları ve libmetal ön derleme adımını ekler."""
    pre = re.search(r'prebuildStep="([^"]*)"', amp_cproject)
    if pre is None:
        raise SystemExit("TTY_echo .cproject'te prebuildStep yok")
    text = rtos_cproject.replace(OLD_NAME, NEW_NAME)

    includes = OPENAMP_INCLUDES + [fwd(os.path.join(REPO, "m33", "freertos", "App", "Inc")),
                                   fwd(os.path.join(REPO, "m33", "common"))]

    def add_values(match, values):
        head = match.group(0)
        return head + "".join('\n\t\t\t\t\t\t\t\t\t<listOptionValue builtIn="false" value="%s"/>' % v for v in values)

    text, n_inc = re.subn(r'<option [^>]*superClass="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.tool\.c\.compiler\.option\.includepaths"[^>]*>',
                          lambda m: add_values(m, includes), text)
    text, n_def = re.subn(r'<option [^>]*superClass="com\.st\.stm32cube\.ide\.mcu\.gnu\.managedbuild\.tool\.c\.compiler\.option\.definedsymbols"[^>]*>',
                          lambda m: add_values(m, OPENAMP_DEFINES), text)

    def add_prebuild(m):
        tag = m.group(0)
        if "prebuildStep=" in tag:
            return tag
        return tag.replace("<configuration ", '<configuration prebuildStep="%s" ' % pre.group(1), 1)

    text, n_cfg = re.subn(r"<configuration artifactExtension[^>]*>", add_prebuild, text)
    return text, n_inc, n_def, n_cfg


def main():
    ap = argparse.ArgumentParser(description="F7Sup FreeRTOS + OpenAMP projesini olusturur")
    ap.add_argument("--cube", default=DEFAULT_CUBE, help="STM32CubeMP2 paket koku")
    ap.add_argument("--kontrol", action="store_true", help="yalniz denetle, yazma")
    a = ap.parse_args()

    apps = os.path.join(a.cube, APPS)
    rtos = os.path.join(apps, "FreeRTOS", "FreeRTOS_ThreadCreation")
    amp = os.path.join(apps, "OpenAMP", "OpenAMP_TTY_echo")
    dest = os.path.join(apps, "F7Sup", "F7Sup_FreeRTOS")
    for p in (rtos, amp):
        if not os.path.isdir(p):
            raise SystemExit("bulunamadi: %s" % p)
    if os.path.exists(dest) and not a.kontrol:
        raise SystemExit("zaten var (once silmen gerekir): %s" % dest)

    with open(os.path.join(rtos, IDE, ".project"), encoding="utf-8") as f:
        rtos_project = f.read()
    with open(os.path.join(amp, IDE, ".project"), encoding="utf-8") as f:
        amp_project = f.read()
    with open(os.path.join(rtos, IDE, ".cproject"), encoding="utf-8") as f:
        rtos_cproject = f.read()
    with open(os.path.join(amp, IDE, ".cproject"), encoding="utf-8") as f:
        amp_cproject = f.read()

    project, n_links = build_project_xml(rtos_project, amp_project)
    cproject, n_inc, n_def, n_cfg = build_cproject_xml(rtos_cproject, amp_cproject)
    ET.fromstring(project)                                   # bozuk XML yazılmasın
    ET.fromstring(cproject.split("?>", 1)[1] if cproject.startswith("<?xml") else cproject)

    # Bütün bağlantılar yeni proje yerinden çözülünce gerçek bir dosyaya mı varıyor?
    proj_dir = os.path.join(dest, IDE)
    missing = []
    for m in re.finditer(r"<locationURI>(PARENT-\d+-PROJECT_LOC/[^<]+)</locationURI>", project):
        path = resolve(m.group(1), proj_dir)
        rel_to_dest = os.path.relpath(path, dest)
        exists = os.path.exists(path) or os.path.exists(os.path.join(rtos, rel_to_dest)) or \
            os.path.exists(os.path.join(amp, rel_to_dest))       # kopyadan sonra oluşacaklar
        if not exists:
            missing.append(m.group(1))
    for m in re.finditer(r"<location>([^<]+)</location>", project):
        if not os.path.exists(m.group(1)):
            missing.append(m.group(1))

    print("eklenen baglanti: %d, include eklenen secenek: %d, define eklenen secenek: %d, yapilandirma: %d"
          % (n_links, n_inc, n_def, n_cfg))
    if missing:
        print("COZULEMEYEN BAGLANTILAR:")
        for m in missing:
            print("  " + m)
        raise SystemExit(1)
    print("butun baglantilar cozuluyor")
    if a.kontrol:
        print("kontrol modu: hicbir sey yazilmadi")
        return

    # Kopya: derleme çıktıları hariç
    shutil.copytree(rtos, dest, ignore=shutil.ignore_patterns("CA35TDCID_*", "Debug", "*.launch"))
    shutil.copytree(os.path.join(amp, "CM33", "NonSecure", "OPENAMP"), os.path.join(dest, "CM33", "NonSecure", "OPENAMP"))
    with open(os.path.join(proj_dir, ".project"), "w", encoding="utf-8", newline="\n") as f:
        f.write(project)
    with open(os.path.join(proj_dir, ".cproject"), "w", encoding="utf-8", newline="\n") as f:
        f.write(cproject)
    print("olusturuldu: %s" % dest)
    print("CubeIDE: File > Import > Existing Projects into Workspace > %s" % proj_dir)


if __name__ == "__main__":
    sys.exit(main())
