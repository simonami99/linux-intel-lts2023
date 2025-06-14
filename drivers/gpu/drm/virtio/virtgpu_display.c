/*
 * Copyright (C) 2015 Red Hat, Inc.
 * All Rights Reserved.
 *
 * Authors:
 *    Dave Airlie
 *    Alon Levy
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER(S) OR AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_damage_helper.h>
#include <drm/drm_edid.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>
#include <drm/display/drm_hdcp_helper.h>

#include "virtgpu_drv.h"

#define XRES_MIN    32
#define YRES_MIN    32

#define XRES_DEF  1024
#define YRES_DEF   768

#define XRES_MAX  8192
#define YRES_MAX  8192

#define drm_connector_to_virtio_gpu_output(x) \
	container_of(x, struct virtio_gpu_output, conn)
#define hdcp_to_virtio_gpu_output(x) \
	container_of(x, struct virtio_gpu_output, hdcp)

static int virtio_irq_enable_vblank(struct drm_crtc *crtc)
{

	struct drm_device *dev = crtc->dev;
	struct virtio_gpu_device *vgdev = dev->dev_private;
	struct virtio_gpu_output *output = drm_crtc_to_virtio_gpu_output(crtc);

	do {
		virtio_gpu_vblank_poll_arm(vgdev->vblank[output->index].vblank.vq);
	} while (!virtqueue_enable_cb(vgdev->vblank[output->index].vblank.vq));
	return 0;
}

static void virtio_irq_disable_vblank(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct virtio_gpu_device *vgdev;
	vgdev = dev->dev_private;
	struct virtio_gpu_output *output = drm_crtc_to_virtio_gpu_output(crtc);

	virtqueue_disable_cb(vgdev->vblank[output->index].vblank.vq);
}

static const struct drm_crtc_funcs virtio_gpu_crtc_funcs = {
	.set_config             = drm_atomic_helper_set_config,
	.destroy                = drm_crtc_cleanup,

	.page_flip              = drm_atomic_helper_page_flip,
	.reset                  = drm_atomic_helper_crtc_reset,
	.atomic_duplicate_state = drm_atomic_helper_crtc_duplicate_state,
	.atomic_destroy_state   = drm_atomic_helper_crtc_destroy_state,
	.enable_vblank = virtio_irq_enable_vblank,
	.disable_vblank = virtio_irq_disable_vblank,
};

static const struct drm_framebuffer_funcs virtio_gpu_fb_funcs = {
	.create_handle = drm_gem_fb_create_handle,
	.destroy = drm_gem_fb_destroy,
	.dirty = drm_atomic_helper_dirtyfb,
};

static void virtio_gpu_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct virtio_gpu_device *vgdev = dev->dev_private;
	struct virtio_gpu_output *output = drm_crtc_to_virtio_gpu_output(crtc);

	virtio_gpu_cmd_set_scanout(vgdev, output->index, 0,
				   crtc->mode.hdisplay,
				   crtc->mode.vdisplay, 0, 0);
	virtio_gpu_notify(vgdev);
}

static void virtio_gpu_crtc_atomic_enable(struct drm_crtc *crtc,
					  struct drm_atomic_state *state)
{
	struct drm_device *dev = crtc->dev;
	struct virtio_gpu_device *vgdev = dev->dev_private;
	if(vgdev->has_vblank) {
		drm_crtc_vblank_on(crtc);
	}
}

static void virtio_gpu_crtc_atomic_disable(struct drm_crtc *crtc,
					   struct drm_atomic_state *state)
{
	struct drm_device *dev = crtc->dev;
	struct virtio_gpu_device *vgdev = dev->dev_private;
	struct virtio_gpu_output *output = drm_crtc_to_virtio_gpu_output(crtc);
	const unsigned pipe = drm_crtc_index(crtc);

	struct drm_pending_vblank_event *e = xchg(&vgdev->cache_event[pipe], NULL);
	/* Send cached event even it's still premature. */
	if (e) {
		spin_lock_irq(&dev->event_lock);
		drm_crtc_send_vblank_event(crtc, e);
		spin_unlock_irq(&dev->event_lock);
		drm_crtc_vblank_put(crtc);
	}

	if (vgdev->has_vblank) {
		drm_crtc_vblank_off(crtc);
	}

	virtio_gpu_cmd_set_scanout(vgdev, output->index, 0, 0, 0, 0, 0);
	virtio_gpu_notify(vgdev);
}

static void virtio_gpu_crtc_atomic_begin(struct drm_crtc *crtc,
					 struct drm_atomic_state *state)
{
	struct virtio_gpu_device *vgdev = crtc->dev->dev_private;
	struct drm_device *drm = crtc->dev;
	const unsigned pipe = drm_crtc_index(crtc);
	struct drm_pending_vblank_event *old_e, *e = crtc->state->event;

	if (!vgdev->has_vblank || !crtc->state->event)
		return;

	if (drm_crtc_vblank_get(crtc)) {
		/* Cannot enable vblank, send it right now. */
		spin_lock_irq(&drm->event_lock);
		drm_crtc_send_vblank_event(crtc, e);
		spin_unlock_irq(&drm->event_lock);
		crtc->state->event = NULL;
		return;
	}

	if (!vgdev->has_flip_sequence) {
		spin_lock_irq(&drm->event_lock);
		/* Let drm_handle_vblank signal it later in the vblank interrupt
		 * and the vblank refcount will be released at that time. */
		drm_crtc_arm_vblank_event(crtc, e);
		spin_unlock_irq(&drm->event_lock);
	} else {
		crtc->state->event->sequence =
			atomic64_read(&vgdev->flip_sequence[pipe]) + 1;
		old_e = xchg(&vgdev->cache_event[pipe], crtc->state->event);
		if (old_e) {
			spin_lock_irq(&drm->event_lock);
			drm_crtc_send_vblank_event(crtc, old_e);
			spin_unlock_irq(&drm->event_lock);
			drm_crtc_vblank_put(crtc);
		}
	}
	crtc->state->event = NULL;
}

static int virtio_gpu_crtc_atomic_check(struct drm_crtc *crtc,
					struct drm_atomic_state *state)
{
	struct virtio_gpu_output *output = NULL;
	struct drm_device *dev = crtc->dev;
	int num_scalers_need;
	struct virtio_gpu_device *vgdev = dev->dev_private;

	output = drm_crtc_to_virtio_gpu_output(crtc);
	if(vgdev->has_scaling) {
		num_scalers_need = hweight32(output->scaler_users);
		if(num_scalers_need > SKL_NUM_SCALERS) {
			drm_dbg_kms(dev, "Too many scaling requests %d > %d\n", num_scalers_need, SKL_NUM_SCALERS);
			output->scaler_users = 0;
			return -EINVAL;
		}
	}
	return 0;
}

static void virtio_gpu_resource_flush_sync(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct virtio_gpu_device *vgdev = dev->dev_private;
	struct virtio_gpu_output *output = drm_crtc_to_virtio_gpu_output(crtc);
	virtio_gpu_cmd_flush_sync(vgdev, output->index);
	virtio_gpu_notify(vgdev);
}

static void virtio_gpu_crtc_atomic_flush(struct drm_crtc *crtc,
					 struct drm_atomic_state *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
									  crtc);
	struct virtio_gpu_output *output = drm_crtc_to_virtio_gpu_output(crtc);
	struct drm_device *drm = crtc->dev;
	struct virtio_gpu_device *vgdev = drm->dev_private;
	const unsigned pipe = drm_crtc_index(crtc);

	if(vgdev->has_multi_plane)
		virtio_gpu_resource_flush_sync(crtc);

	if(vgdev->has_scaling)
		output->scaler_users = 0;

	/*
	 * virtio-gpu can't do modeset and plane update operations
	 * independent from each other.  So the actual modeset happens
	 * in the plane update callback, and here we just check
	 * whenever we must force the modeset.
	 */
	if (drm_atomic_crtc_needs_modeset(crtc_state)) {
		output->needs_modeset = true;
	}
}

static const struct drm_crtc_helper_funcs virtio_gpu_crtc_helper_funcs = {
	.mode_set_nofb = virtio_gpu_crtc_mode_set_nofb,
	.atomic_begin  = virtio_gpu_crtc_atomic_begin,
	.atomic_check  = virtio_gpu_crtc_atomic_check,
	.atomic_flush  = virtio_gpu_crtc_atomic_flush,
	.atomic_enable = virtio_gpu_crtc_atomic_enable,
	.atomic_disable = virtio_gpu_crtc_atomic_disable,
};

static void virtio_gpu_enc_mode_set(struct drm_encoder *encoder,
				    struct drm_display_mode *mode,
				    struct drm_display_mode *adjusted_mode)
{
}

static void virtio_gpu_enc_enable(struct drm_encoder *encoder)
{
}

static void virtio_gpu_enc_disable(struct drm_encoder *encoder)
{
}

static int virtio_gpu_conn_get_modes(struct drm_connector *connector)
{
	struct virtio_gpu_output *output =
		drm_connector_to_virtio_gpu_output(connector);
	struct drm_display_mode *mode = NULL;
	int count, width, height;

	if (output->edid) {
		count = drm_add_edid_modes(connector, output->edid);
		if (count)
			return count;
	}

	width  = le32_to_cpu(output->info.r.width);
	height = le32_to_cpu(output->info.r.height);
	count = drm_add_modes_noedid(connector, XRES_MAX, YRES_MAX);

	if (width == 0 || height == 0) {
		drm_set_preferred_mode(connector, XRES_DEF, YRES_DEF);
	} else {
		DRM_DEBUG("add mode: %dx%d\n", width, height);
		mode = drm_cvt_mode(connector->dev, width, height, 60,
				    false, false, false);
		if (!mode)
			return count;
		mode->type |= DRM_MODE_TYPE_PREFERRED;
		drm_mode_probed_add(connector, mode);
		count++;
	}

	return count;
}

static enum drm_mode_status virtio_gpu_conn_mode_valid(struct drm_connector *connector,
				      struct drm_display_mode *mode)
{
	struct virtio_gpu_output *output =
		drm_connector_to_virtio_gpu_output(connector);
	int width, height;

	width  = le32_to_cpu(output->info.r.width);
	height = le32_to_cpu(output->info.r.height);

	if (!(mode->type & DRM_MODE_TYPE_PREFERRED))
		return MODE_OK;
	if (mode->hdisplay == XRES_DEF && mode->vdisplay == YRES_DEF)
		return MODE_OK;
	if (mode->hdisplay <= width  && mode->hdisplay >= width - 16 &&
	    mode->vdisplay <= height && mode->vdisplay >= height - 16)
		return MODE_OK;

	DRM_DEBUG("del mode: %dx%d\n", mode->hdisplay, mode->vdisplay);
	return MODE_BAD;
}

static const struct drm_encoder_helper_funcs virtio_gpu_enc_helper_funcs = {
	.mode_set   = virtio_gpu_enc_mode_set,
	.enable     = virtio_gpu_enc_enable,
	.disable    = virtio_gpu_enc_disable,
};

static const struct drm_connector_helper_funcs virtio_gpu_conn_helper_funcs = {
	.get_modes    = virtio_gpu_conn_get_modes,
	.mode_valid   = virtio_gpu_conn_mode_valid,
};

static enum drm_connector_status virtio_gpu_conn_detect(
			struct drm_connector *connector,
			bool force)
{
	struct virtio_gpu_output *output =
		drm_connector_to_virtio_gpu_output(connector);

	if (output->info.enabled)
		return connector_status_connected;
	else
		return connector_status_disconnected;
}

static void virtio_gpu_conn_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static const struct drm_connector_funcs virtio_gpu_connector_funcs = {
	.detect = virtio_gpu_conn_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = virtio_gpu_conn_destroy,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static void virtio_gpu_hdcp_prop_work(struct work_struct *work)
{
	struct virtio_gpu_hdcp *hdcp = container_of(work, struct virtio_gpu_hdcp, prop_work);
	struct virtio_gpu_output *output = hdcp_to_virtio_gpu_output(hdcp);
	struct drm_connector *connector = &output->conn;

	DRM_DEBUG("virtio gpu hdcp prop work\n");
	if (!connector || !connector->dev)
	       return;
	drm_modeset_lock(&connector->dev->mode_config.connection_mutex, NULL);
	mutex_lock(&hdcp->mutex);
	if (hdcp->value != DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
		drm_hdcp_update_content_protection(connector, hdcp->value);
	}
	mutex_unlock(&hdcp->mutex);
	drm_modeset_unlock(&connector->dev->mode_config.connection_mutex);
	drm_connector_put(connector);
}

static void vgdev_hdcp_init(struct virtio_gpu_device *vgdev, struct virtio_gpu_hdcp *hdcp, int index)
{
	struct virtio_gpu_output *output = hdcp_to_virtio_gpu_output(hdcp);
	struct drm_connector *connector = &output->conn;
	virtio_gpu_cmd_cp_query(vgdev, index);
	if (hdcp->hdcp) {
		drm_connector_attach_content_protection_property(connector, hdcp->hdcp2);
		hdcp->value = 0;
		hdcp->type = 0;
		DRM_DEBUG("virtio gpu hdcp init, hdcp2:%d\n", hdcp->hdcp2);

		virtio_gpu_cmd_cp_set(vgdev, index, hdcp->value, hdcp->type);
		mutex_init(&hdcp->mutex);
		INIT_WORK(&hdcp->prop_work, virtio_gpu_hdcp_prop_work);
	}
}

static int vgdev_output_init(struct virtio_gpu_device *vgdev, int index)
{
	struct drm_device *dev = vgdev->ddev;
	struct virtio_gpu_output *output = vgdev->outputs + index;
	struct drm_connector *connector = &output->conn;
	struct drm_encoder *encoder = &output->enc;
	struct drm_crtc *crtc = &output->crtc;
	struct drm_plane *primary, *cursor;

	output->index = index;
	if (index == 0) {
		output->info.enabled = cpu_to_le32(true);
		output->info.r.width = cpu_to_le32(XRES_DEF);
		output->info.r.height = cpu_to_le32(YRES_DEF);
	}
	output->plane_idx_offset = -1;

	if(vgdev->has_scaling)
		output->scaler_users = 0;
	primary = virtio_gpu_plane_init(vgdev, DRM_PLANE_TYPE_PRIMARY, index);
	if (IS_ERR(primary))
		return PTR_ERR(primary);
	cursor = virtio_gpu_plane_init(vgdev, DRM_PLANE_TYPE_CURSOR, index);
	if (IS_ERR(cursor))
		return PTR_ERR(cursor);

	if(vgdev->has_multi_plane) {
		struct drm_plane *sprite;
		int i;
		for(i=0; i< vgdev->outputs[index].plane_num; i++) {
			sprite = virtio_gpu_plane_init(vgdev, DRM_PLANE_TYPE_OVERLAY, index);
			if (IS_ERR(sprite))
				return PTR_ERR(sprite);
		}
	}

	drm_crtc_init_with_planes(dev, crtc, primary, cursor,
				  &virtio_gpu_crtc_funcs, NULL);
	drm_crtc_helper_add(crtc, &virtio_gpu_crtc_helper_funcs);

	drm_connector_init(dev, connector, &virtio_gpu_connector_funcs,
			   DRM_MODE_CONNECTOR_VIRTUAL);
	drm_connector_helper_add(connector, &virtio_gpu_conn_helper_funcs);
	if (vgdev->has_edid)
		drm_connector_attach_edid_property(connector);

	drm_simple_encoder_init(dev, encoder, DRM_MODE_ENCODER_VIRTUAL);
	drm_encoder_helper_add(encoder, &virtio_gpu_enc_helper_funcs);
	encoder->possible_crtcs = 1 << index;

	drm_connector_attach_encoder(connector, encoder);
	if (vgdev->has_hdcp)
		vgdev_hdcp_init(vgdev, &output->hdcp, index);
	drm_connector_register(connector);
	return 0;
}

static struct drm_framebuffer *
virtio_gpu_user_framebuffer_create(struct drm_device *dev,
				   struct drm_file *file_priv,
				   const struct drm_mode_fb_cmd2 *mode_cmd)
{
	struct drm_gem_object *obj = NULL;
	struct virtio_gpu_framebuffer *virtio_gpu_fb;

	const struct drm_format_info *info;
	struct drm_gem_object *objs[DRM_FORMAT_MAX_PLANES];

	unsigned int i;
	int ret;

	info = drm_get_format_info(dev, mode_cmd);
	if (!info) {
		drm_dbg_kms(dev, "Failed to get FB format info\n");
		return ERR_PTR(-EINVAL);
	}

	virtio_gpu_fb = kzalloc(sizeof(*virtio_gpu_fb), GFP_KERNEL);
	if (virtio_gpu_fb == NULL) {
		return ERR_PTR(-ENOMEM);
	}

	for (i = 0; i < info->num_planes; i++) {
		objs[i] = drm_gem_object_lookup(file_priv, mode_cmd->handles[i]);
		if (!objs[i]) {
			drm_dbg_kms(dev, "Failed to lookup GEM object\n");
			goto error;
		}
		virtio_gpu_fb->base.obj[i] = objs[i];
	}

	drm_helper_mode_fill_fb_struct(dev, &virtio_gpu_fb->base, mode_cmd);
	ret = drm_framebuffer_init(dev, &virtio_gpu_fb->base, &virtio_gpu_fb_funcs);
	if (ret)
		goto error;

	return &virtio_gpu_fb->base;
error:
	kfree(virtio_gpu_fb);
	while (i > 0) {
		--i;
		drm_gem_object_put(objs[i]);
	}
	return ERR_PTR(ret);
}

static const struct drm_mode_config_funcs virtio_gpu_mode_funcs = {
	.fb_create = virtio_gpu_user_framebuffer_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static void
virtio_gpu_wait_for_vblanks(struct drm_device *dev,
			    struct drm_atomic_state *old_state)
{
	struct virtio_gpu_device *vgdev = dev->dev_private;
	struct drm_crtc *crtc;
	struct drm_crtc_state *old_crtc_state, *new_crtc_state;
	int i, ret;
	unsigned int crtc_mask = 0;

	 /*
	  * Legacy cursor ioctls are completely unsynced, and userspace
	  * relies on that (by doing tons of cursor updates).
	  */
	if (old_state->legacy_cursor_update)
		return;

	for_each_oldnew_crtc_in_state(old_state, crtc, old_crtc_state, new_crtc_state, i) {
		if (!new_crtc_state->active)
			continue;

		ret = drm_crtc_vblank_get(crtc);
		if (ret != 0)
			continue;

		crtc_mask |= drm_crtc_mask(crtc);
		old_state->crtcs[i].last_vblank_count =
			vgdev->has_vblank && vgdev->has_flip_sequence ?
			atomic64_read(&vgdev->flip_sequence[i]) :
			drm_crtc_vblank_count(crtc);
	}

	for_each_old_crtc_in_state(old_state, crtc, old_crtc_state, i) {
		if (!(crtc_mask & drm_crtc_mask(crtc)))
			continue;

		ret = wait_event_timeout(dev->vblank[i].queue,
				old_state->crtcs[i].last_vblank_count !=
					(vgdev->has_vblank && vgdev->has_flip_sequence ?
						atomic64_read(&vgdev->flip_sequence[i]) :
						drm_crtc_vblank_count(crtc)),
				msecs_to_jiffies(100));

		WARN(!ret, "[CRTC:%d:%s] vblank wait timed out\n",
		     crtc->base.id, crtc->name);

		drm_crtc_vblank_put(crtc);
	}
}

static void virtio_gpu_hdcp_atomic_process(struct drm_atomic_state *state)
{
	struct drm_connector *connector;
	struct drm_connector_state *old_con_state, *new_con_state;
	struct virtio_gpu_output *output;
	struct virtio_gpu_device *vgdev;
	uint32_t i;
	for_each_oldnew_connector_in_state(state, connector, old_con_state, new_con_state, i) {
		output = drm_connector_to_virtio_gpu_output(connector);
		vgdev = connector->dev->dev_private;
		if (vgdev->has_hdcp && output->hdcp.hdcp && (old_con_state->hdcp_content_type != new_con_state->hdcp_content_type ||
				old_con_state->content_protection != new_con_state->content_protection)) {
			DRM_DEBUG("hdcp state changed, send cmd to host, type:%d,cp:%d\n",
							new_con_state->hdcp_content_type, new_con_state->content_protection);

			if (new_con_state->content_protection == DRM_MODE_CONTENT_PROTECTION_UNDESIRED) {
				mutex_lock(&output->hdcp.mutex);
				output->hdcp.value = DRM_MODE_CONTENT_PROTECTION_UNDESIRED;
				mutex_unlock(&output->hdcp.mutex);
			}

			virtio_gpu_cmd_cp_set(vgdev, output->index, new_con_state->hdcp_content_type, new_con_state->content_protection);
		}
	}
}

static void virtio_gpu_commit_tail(struct drm_atomic_state *old_state)
{
	struct drm_device *dev = old_state->dev;

	virtio_gpu_hdcp_atomic_process(old_state);

	drm_atomic_helper_commit_modeset_disables(dev, old_state);

	drm_atomic_helper_commit_planes(dev, old_state, 0);

	drm_atomic_helper_commit_modeset_enables(dev, old_state);

	drm_atomic_helper_fake_vblank(old_state);

	drm_atomic_helper_commit_hw_done(old_state);

	virtio_gpu_wait_for_vblanks(dev, old_state);

	drm_atomic_helper_cleanup_planes(dev, old_state);
}

static struct drm_mode_config_helper_funcs virtgio_gpu_mode_config_helpers = {
	.atomic_commit_tail = virtio_gpu_commit_tail,
};

int virtio_gpu_modeset_init(struct virtio_gpu_device *vgdev)
{
	int i, ret;

	if (!vgdev->num_scanouts)
		return 0;

	ret = drmm_mode_config_init(vgdev->ddev);
	if (ret)
		return ret;

	vgdev->ddev->mode_config.funcs = &virtio_gpu_mode_funcs;
	vgdev->ddev->mode_config.helper_private = &virtgio_gpu_mode_config_helpers;

	/* modes will be validated against the framebuffer size */
	vgdev->ddev->mode_config.min_width = XRES_MIN;
	vgdev->ddev->mode_config.min_height = YRES_MIN;
	vgdev->ddev->mode_config.max_width = XRES_MAX;
	vgdev->ddev->mode_config.max_height = YRES_MAX;

	if (!vgdev->has_modifier)
		vgdev->ddev->mode_config.fb_modifiers_not_supported = true;

	for (i = 0 ; i < vgdev->num_scanouts; ++i)
		vgdev_output_init(vgdev, i);

	drm_mode_config_reset(vgdev->ddev);
	return 0;
}

void virtio_gpu_modeset_fini(struct virtio_gpu_device *vgdev)
{
	int i;

	if (!vgdev->num_scanouts)
		return;

	for (i = 0 ; i < vgdev->num_scanouts; ++i)
		kfree(vgdev->outputs[i].edid);
}
