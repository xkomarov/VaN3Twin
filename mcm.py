import asn1tools as asn
import glob
import os

asn_files = glob.glob(os.path.join(security_folder, "*.asn"))

mcm = asn.compile_files(mcm_asn, "uper")

print(mcm)
