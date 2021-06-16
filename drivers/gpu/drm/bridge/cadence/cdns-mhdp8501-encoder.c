#include <linux/component.h>
#include <linux/platform_device.h>

#include <drm/drm_of.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_bridge_connector.h>

#include "cdns-mhdp8501-core.h"

static int cdns_mhdp8501_bind(struct device *dev, struct device *master, void *data)
{
	struct cdns_mhdp8501_device *mhdp = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	int ret;

	mhdp->encoder.possible_crtcs = drm_of_find_possible_crtcs(drm,
								  dev->of_node);

	ret = drm_simple_encoder_init(drm, &mhdp->encoder,
				      DRM_MODE_ENCODER_TMDS);
	if (ret)
		goto err_encoder;

	ret = drm_bridge_attach(&mhdp->encoder, &mhdp->bridge, NULL,
				DRM_BRIDGE_ATTACH_NO_CONNECTOR);
	if (ret)
		goto err_bridge;

	mhdp->connector = drm_bridge_connector_init(drm, &mhdp->encoder);
	if (IS_ERR(mhdp->connector)) {
		ret = PTR_ERR(mhdp->connector);
		goto err_bridge;
	}
	drm_connector_attach_encoder(mhdp->connector, &mhdp->encoder);

	return 0;

err_bridge:
	drm_encoder_cleanup(&mhdp->encoder);
err_encoder:
	return ret;
}

static void cdns_mhdp8501_unbind(struct device *dev, struct device *master,
			     void *data)
{
	struct cdns_mhdp8501_device *mhdp = dev_get_drvdata(dev);

	/* TODO(mw) */
	kfree(&mhdp->connector);
	drm_encoder_cleanup(&mhdp->encoder);
}

static const struct component_ops cdns_mhdp8501_ops = {
	.bind = cdns_mhdp8501_bind,
	.unbind = cdns_mhdp8501_unbind,
};

int cdns_mhdp8501_encoder_init(struct platform_device *pdev)
{
	return component_add(&pdev->dev, &cdns_mhdp8501_ops);
}
