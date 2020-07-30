. ~/esp-idf/export.sh
. build-venv/bin/activate
make USER_C_MODULES=../../modules CFLAGS_EXTRA=-DMODULE_OUTPUT_ENABLED=1 all
make deploy BAUD=115200
