EXTRA_CFLAGS += -D_RESET_IP_ -DLINUX_ENV -D_DISABLE_REORDER_ -D_USE_INT_ -I$(shell pwd)/drivers/misc/codec264/Decoder -I$(shell pwd)/drivers/misc/codec264  \
                -I$(shell pwd)/drivers/misc/codec264/Encoder -I$(shell pwd)/drivers/misc/codec264

obj-$(CONFIG_W55FA92_VDE) += favc_module.o Decoder/favc_dec_opt.o Encoder/favc_enc_opt.o \
                 Decoder/AVCdec.o Decoder/decoder.o \
                 Decoder/h264_funct.o  Decoder/qsort.o \
                 Encoder/bs.o Encoder/common.o Encoder/encoder.o \
                 Encoder/sequence.o Encoder/slice.o nv_mem.o Encoder/memory.o