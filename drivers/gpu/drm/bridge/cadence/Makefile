# SPDX-License-Identifier: GPL-2.0-only
obj-$(CONFIG_DRM_CDNS_MHDP_COMMON) += cdns-mhdp-mbox.o cdns-mhdp-misc.o

obj-$(CONFIG_DRM_CDNS_MHDP8546) += cdns-mhdp8546.o
cdns-mhdp8546-y := cdns-mhdp8546-core.o cdns-mhdp8546-hdcp.o
cdns-mhdp8546-$(CONFIG_DRM_CDNS_MHDP8546_J721E) += cdns-mhdp8546-j721e.o

obj-$(CONFIG_DRM_CDNS_MHDP8501) += cdns-mhdp8501.o
cdns-mhdp8501-y := cdns-mhdp8501-core.o cdns-mhdp8501-afe.o cdns-mhdp8501-encoder.o
