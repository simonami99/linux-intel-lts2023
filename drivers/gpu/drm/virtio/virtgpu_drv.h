/*
 * Copyright (C) 2015 Red Hat, Inc.
 * All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial
 * portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE COPYRIGHT OWNER(S) AND/OR ITS SUPPLIERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef VIRTIO_DRV_H
#define VIRTIO_DRV_H

#include <linux/dma-direction.h>
#include <linux/virtio.h>
#include <linux/virtio_ids.h>
#include <linux/virtio_config.h>
#include <linux/virtio_gpu.h>

#include <drm/drm_atomic.h>
#include <drm/drm_drv.h>
#include <drm/drm_encoder.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem.h>
#include <drm/drm_gem_shmem_helper.h>
#include <drm/drm_ioctl.h>
#include <drm/drm_probe_helper.h>
#include <drm/virtgpu_drm.h>



#define DRIVER_NAME "virtio_gpu"
#define DRIVER_DESC "virtio GPU"
#define DRIVER_DATE "0"

#define DRIVER_MAJOR 0
#define DRIVER_MINOR 1
#define DRIVER_PATCHLEVEL 0

#define STATE_INITIALIZING 0
#define STATE_OK 1
#define STATE_ERR 2

#define MAX_CAPSET_ID 63
#define MAX_RINGS 64

struct virtio_gpu_object_params {
	unsigned long size;
	bool dumb;
	/* 3d */
	bool virgl;
	bool blob;

	/* classic resources only */
	uint32_t format;
	uint32_t width;
	uint32_t height;
	uint32_t target;
	uint32_t bind;
	uint32_t depth;
	uint32_t array_size;
	uint32_t last_level;
	uint32_t nr_samples;
	uint32_t flags;

	/* blob resources only */
	uint32_t ctx_id;
	uint32_t blob_mem;
	uint32_t blob_flags;
	uint64_t blob_id;
	bool protected;
};

struct virtio_gpu_object {
	struct drm_gem_shmem_object base;
	uint32_t hw_res_handle;
	bool dumb;
	bool prime;
	bool created;
	bool host3d_blob, guest_blob;
	uint32_t blob_mem, blob_flags;

	int uuid_state;
	uuid_t uuid;
	/* indicating the obj in lmem or system memory */
	int locate;
	/* Address cache for prime object */
	struct virtio_gpu_mem_entry *ents;
	uint32_t nents;
	bool protected;
};
#define gem_to_virtio_gpu_obj(gobj) \
	container_of((gobj), struct virtio_gpu_object, base.base)

struct virtio_gpu_object_shmem {
	struct virtio_gpu_object base;
};

struct virtio_gpu_object_vram {
	struct virtio_gpu_object base;
	uint32_t map_state;
	uint32_t map_info;
	struct drm_mm_node vram_node;
};

#define to_virtio_gpu_shmem(virtio_gpu_object) \
	container_of((virtio_gpu_object), struct virtio_gpu_object_shmem, base)

#define to_virtio_gpu_vram(virtio_gpu_object) \
	container_of((virtio_gpu_object), struct virtio_gpu_object_vram, base)

struct virtio_gpu_object_array {
	struct ww_acquire_ctx ticket;
	struct list_head next;
	u32 nents, total;
	struct drm_gem_object *objs[];
};

struct virtio_gpu_object_restore {
	struct virtio_gpu_object *bo;
	struct virtio_gpu_object_params params;
	struct list_head node;
};

struct virtio_gpu_vbuffer;
struct virtio_gpu_device;

typedef void (*virtio_gpu_resp_cb)(struct virtio_gpu_device *vgdev,
				   struct virtio_gpu_vbuffer *vbuf);

struct virtio_gpu_fence_driver {
	atomic64_t       last_fence_id;
	uint64_t         current_fence_id;
	uint64_t         context;
	struct list_head fences;
	spinlock_t       lock;
};

struct virtio_gpu_fence_event {
	struct drm_pending_event base;
	struct drm_event event;
};

struct virtio_gpu_fence {
	struct dma_fence f;
	uint32_t ring_idx;
	uint64_t fence_id;
	bool emit_fence_info;
	struct virtio_gpu_fence_event *e;
	struct virtio_gpu_fence_driver *drv;
	struct list_head node;
};

struct virtio_gpu_vbuffer {
	char *buf;
	int size;

	void *data_buf;
	uint32_t data_size;

	char *resp_buf;
	int resp_size;
	virtio_gpu_resp_cb resp_cb;
	void *resp_cb_data;

	struct virtio_gpu_object_array *objs;
	struct list_head list;

	uint32_t seqno;
};

struct virtio_gpu_hdcp {
	struct work_struct prop_work;
	struct mutex mutex;
	struct virtio_gpu_output *output;
	u32 value;
	u32 type;
	int hdcp2;
	int connector_hdcp2;
	int hdcp;
	bool query_done;
};

#define VIRTIO_GPU_MAX_PLANES 6
/*hardcode igpu scaler number ver>11 */
#define SKL_NUM_SCALERS 2

#define VBLANK_EVENT_CACHE_SIZE	3

struct virtio_gpu_output {
	int index;
	struct drm_crtc crtc;
	struct drm_connector conn;
	struct drm_encoder enc;
	struct virtio_gpu_display_one info;
	struct virtio_gpu_update_cursor cursor;
	struct edid *edid;
	int cur_x;
	int cur_y;
	bool needs_modeset;
	int plane_num;
	int plane_idx_offset;
	uint64_t rotation[VIRTIO_GPU_MAX_PLANES];
	unsigned scaler_users;
	struct virtio_gpu_hdcp hdcp;
};
#define drm_crtc_to_virtio_gpu_output(x) \
	container_of(x, struct virtio_gpu_output, crtc)

struct virtio_gpu_framebuffer {
	struct drm_framebuffer base;
};
#define to_virtio_gpu_framebuffer(x) \
	container_of(x, struct virtio_gpu_framebuffer, base)

struct virtio_gpu_queue {
	struct virtqueue *vq;
	spinlock_t qlock;
	wait_queue_head_t ack_queue;
	struct work_struct dequeue_work;
	uint32_t seqno;
};

struct virtio_gpu_drv_capset {
	uint32_t id;
	uint32_t max_version;
	uint32_t max_size;
};

struct virtio_gpu_drv_cap_cache {
	struct list_head head;
	void *caps_cache;
	uint32_t id;
	uint32_t version;
	uint32_t size;
	atomic_t is_valid;
};

struct virtio_gpu_vblank {
	struct virtio_gpu_queue vblank;
	uint32_t buf[4];
};

static inline bool drm_vblank_passed(u64 seq, u64 ref)
{
	return (seq - ref) <= (1 << 23);
}

struct virtio_gpu_device {
	struct drm_device *ddev;

	struct virtio_device *vdev;

	struct virtio_gpu_output outputs[VIRTIO_GPU_MAX_SCANOUTS];
	struct drm_pending_vblank_event *cache_event[VIRTIO_GPU_MAX_SCANOUTS];
	atomic64_t flip_sequence[VIRTIO_GPU_MAX_SCANOUTS];
	uint32_t num_scanouts;
	uint32_t num_vblankq;
	/* Setting '1' indicates the spcecific scanout is for dgpu output*/
	uint32_t output_cap_mask;
	struct virtio_gpu_queue ctrlq;
	struct virtio_gpu_queue cursorq;
	struct virtio_gpu_queue hdcpq;
	struct virtio_gpu_cp_notification hdcp_buf;

	struct kmem_cache *vbufs;

	atomic_t pending_commands;

	struct ida	resource_ida;

	wait_queue_head_t resp_wq;
	/* current display info */
	spinlock_t display_info_lock;
	bool display_info_pending;

	struct virtio_gpu_fence_driver fence_drv;

	struct ida	ctx_id_ida;

	bool has_virgl_3d;
	bool has_edid;
	bool has_modifier;
	bool has_scaling;
	bool has_vblank;
	bool has_allow_p2p;
	bool has_flip_sequence;
	bool has_multi_plane;
	bool has_rotation;
	bool has_pixel_blend_mode;
	bool has_multi_planar;
	bool has_hdcp;
	bool has_protected_bo;
	bool has_indirect;
	bool has_resource_assign_uuid;
	bool has_resource_blob;
	bool has_host_visible;
	bool has_context_init;
	struct virtio_shm_region host_visible_region;
	struct drm_mm host_visible_mm;

	struct work_struct config_changed_work;

	struct work_struct obj_free_work;
	spinlock_t obj_free_lock;
	struct list_head obj_free_list;
	struct list_head obj_rec;

	struct virtio_gpu_drv_capset *capsets;
	uint32_t num_capsets;
	uint64_t capset_id_mask;
	struct list_head cap_cache;

	/* protects uuid state when exporting */
	spinlock_t resource_export_lock;
	/* protects map state and host_visible_mm */
	spinlock_t host_visible_lock;
	struct virtio_gpu_vblank vblank[VIRTIO_GPU_MAX_SCANOUTS];
};

struct virtio_gpu_fpriv {
	uint32_t ctx_id;
	uint32_t context_init;
	bool context_created;
	uint32_t num_rings;
	uint64_t base_fence_ctx;
	uint64_t ring_idx_mask;
	struct mutex context_lock;
};

/* virtgpu_ioctl.c */
#define DRM_VIRTIO_NUM_IOCTLS 12
extern struct drm_ioctl_desc virtio_gpu_ioctls[DRM_VIRTIO_NUM_IOCTLS];
void virtio_gpu_create_context(struct drm_device *dev, struct drm_file *file);

/* virtgpu_kms.c */
int virtio_gpu_init(struct virtio_device *vdev, struct drm_device *dev);
void virtio_gpu_deinit(struct drm_device *dev);
void virtio_gpu_release(struct drm_device *dev);
int virtio_gpu_driver_open(struct drm_device *dev, struct drm_file *file);
void virtio_gpu_driver_postclose(struct drm_device *dev, struct drm_file *file);
int virtio_gpu_find_vqs(struct virtio_gpu_device *vgdev);

/* virtgpu_gem.c */
int virtio_gpu_gem_object_open(struct drm_gem_object *obj,
			       struct drm_file *file);
void virtio_gpu_gem_object_close(struct drm_gem_object *obj,
				 struct drm_file *file);
int virtio_gpu_mode_dumb_create(struct drm_file *file_priv,
				struct drm_device *dev,
				struct drm_mode_create_dumb *args);
int virtio_gpu_mode_dumb_mmap(struct drm_file *file_priv,
			      struct drm_device *dev,
			      uint32_t handle, uint64_t *offset_p);

struct virtio_gpu_object_array *virtio_gpu_array_alloc(u32 nents);
struct virtio_gpu_object_array*
virtio_gpu_array_from_handles(struct drm_file *drm_file, u32 *handles, u32 nents);
void virtio_gpu_array_add_obj(struct virtio_gpu_object_array *objs,
			      struct drm_gem_object *obj);
int virtio_gpu_array_lock_resv(struct virtio_gpu_object_array *objs);
void virtio_gpu_array_unlock_resv(struct virtio_gpu_object_array *objs);
void virtio_gpu_array_add_fence(struct virtio_gpu_object_array *objs,
				struct dma_fence *fence);
void virtio_gpu_array_put_free(struct virtio_gpu_object_array *objs);
void virtio_gpu_array_put_free_delayed(struct virtio_gpu_device *vgdev,
				       struct virtio_gpu_object_array *objs);
void virtio_gpu_array_put_free_work(struct work_struct *work);

/* virtgpu_vq.c */
int virtio_gpu_alloc_vbufs(struct virtio_gpu_device *vgdev);
void virtio_gpu_free_vbufs(struct virtio_gpu_device *vgdev);
void virtio_gpu_cmd_create_resource(struct virtio_gpu_device *vgdev,
				    struct virtio_gpu_object *bo,
				    struct virtio_gpu_object_params *params,
				    struct virtio_gpu_object_array *objs,
				    struct virtio_gpu_fence *fence);
void virtio_gpu_cmd_unref_resource(struct virtio_gpu_device *vgdev,
				   struct virtio_gpu_object *bo);
void virtio_gpu_cmd_transfer_to_host_2d(struct virtio_gpu_device *vgdev,
					uint64_t offset,
					uint32_t width, uint32_t height,
					uint32_t x, uint32_t y,
					struct virtio_gpu_object_array *objs,
					struct virtio_gpu_fence *fence);
void virtio_gpu_cmd_resource_flush(struct virtio_gpu_device *vgdev,
				   uint32_t resource_id,
				   uint32_t x, uint32_t y,
				   uint32_t width, uint32_t height,
				   struct virtio_gpu_object_array *objs,
				   struct virtio_gpu_fence *fence);
void virtio_gpu_cmd_set_scanout(struct virtio_gpu_device *vgdev,
				uint32_t scanout_id, uint32_t resource_id,
				uint32_t width, uint32_t height,
				uint32_t x, uint32_t y);

void virtio_gpu_cmd_flush_sync(struct virtio_gpu_device *vgdev,
				   uint32_t scanout_id);

void virtio_gpu_cmd_resource_flush_sprite(struct virtio_gpu_device *vgdev,
				   uint32_t scanout_id,
				   uint32_t plane_indx,
				   struct drm_framebuffer *fb,
				   uint32_t *resource_id,
				   uint32_t resource_cnt,
				   uint32_t x, uint32_t y,
				   uint32_t width, uint32_t height,
				   struct virtio_gpu_object_array *objs,
				   struct virtio_gpu_fence *fence);

void virtio_gpu_object_attach(struct virtio_gpu_device *vgdev,
			      struct virtio_gpu_object *obj,
			      struct virtio_gpu_mem_entry *ents,
			      unsigned int nents);
int virtio_gpu_attach_status_page(struct virtio_gpu_device *vgdev);
int virtio_gpu_detach_status_page(struct virtio_gpu_device *vgdev);
void virtio_gpu_cursor_ping(struct virtio_gpu_device *vgdev,
			    struct virtio_gpu_output *output);
int virtio_gpu_cmd_get_display_info(struct virtio_gpu_device *vgdev);
int virtio_gpu_cmd_get_capset_info(struct virtio_gpu_device *vgdev, int idx);
int virtio_gpu_cmd_get_capset(struct virtio_gpu_device *vgdev,
			      int idx, int version,
			      struct virtio_gpu_drv_cap_cache **cache_p);
int virtio_gpu_cmd_get_edids(struct virtio_gpu_device *vgdev);
void virtio_gpu_cmd_context_create(struct virtio_gpu_device *vgdev, uint32_t id,
				   uint32_t context_init, uint32_t nlen,
				   const char *name);
void virtio_gpu_cmd_context_destroy(struct virtio_gpu_device *vgdev,
				    uint32_t id);
void virtio_gpu_cmd_context_attach_resource(struct virtio_gpu_device *vgdev,
					    uint32_t ctx_id,
					    struct virtio_gpu_object_array *objs);
void virtio_gpu_cmd_context_detach_resource(struct virtio_gpu_device *vgdev,
					    uint32_t ctx_id,
					    struct virtio_gpu_object_array *objs);
void virtio_gpu_cmd_submit(struct virtio_gpu_device *vgdev,
			   void *data, uint32_t data_size,
			   uint32_t ctx_id,
			   struct virtio_gpu_object_array *objs,
			   struct virtio_gpu_fence *fence);
void virtio_gpu_cmd_transfer_from_host_3d(struct virtio_gpu_device *vgdev,
					  uint32_t ctx_id,
					  uint64_t offset, uint32_t level,
					  uint32_t stride,
					  uint32_t layer_stride,
					  struct drm_virtgpu_3d_box *box,
					  struct virtio_gpu_object_array *objs,
					  struct virtio_gpu_fence *fence);
void virtio_gpu_cmd_transfer_to_host_3d(struct virtio_gpu_device *vgdev,
					uint32_t ctx_id,
					uint64_t offset, uint32_t level,
					uint32_t stride,
					uint32_t layer_stride,
					struct drm_virtgpu_3d_box *box,
					struct virtio_gpu_object_array *objs,
					struct virtio_gpu_fence *fence);
void
virtio_gpu_cmd_resource_create_3d(struct virtio_gpu_device *vgdev,
				  struct virtio_gpu_object *bo,
				  struct virtio_gpu_object_params *params,
				  struct virtio_gpu_object_array *objs,
				  struct virtio_gpu_fence *fence);
void virtio_gpu_ctrl_ack(struct virtqueue *vq);
void virtio_gpu_cursor_ack(struct virtqueue *vq);
void virtio_gpu_vblank_ack(struct virtqueue *vq);
void virtio_gpu_hdcp_ack(struct virtqueue *vq);
void virtio_gpu_vblank_poll_arm(struct virtqueue *vq);
void virtio_gpu_fence_ack(struct virtqueue *vq);
void virtio_gpu_dequeue_ctrl_func(struct work_struct *work);
void virtio_gpu_dequeue_cursor_func(struct work_struct *work);
void virtio_gpu_dequeue_hdcp_func(struct work_struct *work);
void virtio_gpu_dequeue_fence_func(struct work_struct *work);
void virtio_gpu_notify(struct virtio_gpu_device *vgdev);
void virtio_gpu_vblankq_notify(struct virtio_gpu_device *vgdev);
void virtio_gpu_hdcp_notify(struct virtio_gpu_device *vgdev);
int
virtio_gpu_cmd_resource_assign_uuid(struct virtio_gpu_device *vgdev,
				    struct virtio_gpu_object_array *objs);

int virtio_gpu_cmd_map(struct virtio_gpu_device *vgdev,
		       struct virtio_gpu_object_array *objs, uint64_t offset);

void virtio_gpu_cmd_unmap(struct virtio_gpu_device *vgdev,
			  struct virtio_gpu_object *bo);

void
virtio_gpu_cmd_resource_create_blob(struct virtio_gpu_device *vgdev,
				    struct virtio_gpu_object *bo,
				    struct virtio_gpu_object_params *params,
				    struct virtio_gpu_mem_entry *ents,
				    uint32_t nents);


int virtio_gpu_cmd_get_planes_info(struct virtio_gpu_device *vgdev, int idx);


int virtio_gpu_cmd_get_plane_rotation(struct virtio_gpu_device *vgdev,
				      uint32_t plane_id, uint32_t scanout_indx);

void
virtio_gpu_cmd_set_scanout_blob(struct virtio_gpu_device *vgdev,
				uint32_t scanout_id,
				struct virtio_gpu_object *bo,
				struct drm_framebuffer *fb,
				uint32_t width, uint32_t height,
				uint32_t x, uint32_t y);
void virtio_gpu_cmd_set_modifier(struct virtio_gpu_device *vgdev,
				 uint32_t scanout_id,
				 struct virtio_gpu_object *bo,
				 struct drm_framebuffer *fb);

void virtio_gpu_cmd_set_scaling(struct virtio_gpu_device *vgdev,
				     uint32_t scanout_id,
				     struct drm_rect *rect_dst);

void virtio_gpu_cmd_send_misc(struct virtio_gpu_device *vgdev, uint32_t scanout_id,
		uint32_t plane_indx, struct virtio_gpu_cmd *cmdp, int cnt);

int virtio_gpu_cmd_cp_set(struct virtio_gpu_device *vgdev,
				uint32_t scanout_id,
				uint32_t type, uint32_t cp);

int virtio_gpu_cmd_cp_query(struct virtio_gpu_device *vgdev,
				uint32_t scanout_id);

/* virtgpu_display.c */
int virtio_gpu_modeset_init(struct virtio_gpu_device *vgdev);
void virtio_gpu_modeset_fini(struct virtio_gpu_device *vgdev);

/* virtgpu_plane.c */
uint32_t virtio_gpu_translate_format(uint32_t drm_fourcc);
struct drm_plane *virtio_gpu_plane_init(struct virtio_gpu_device *vgdev,
					enum drm_plane_type type,
					int index);
void virtio_update_planes_info(int index, int num, u32 *info);

/* virtgpu_fence.c */
struct virtio_gpu_fence *virtio_gpu_fence_alloc(struct virtio_gpu_device *vgdev,
						uint64_t base_fence_ctx,
						uint32_t ring_idx);
void virtio_gpu_fence_emit(struct virtio_gpu_device *vgdev,
			  struct virtio_gpu_ctrl_hdr *cmd_hdr,
			  struct virtio_gpu_fence *fence);
void virtio_gpu_fence_event_process(struct virtio_gpu_device *vdev,
				    u64 fence_id);

/* virtgpu_object.c */
void virtio_gpu_cleanup_object(struct virtio_gpu_object *bo);
struct drm_gem_object *virtio_gpu_create_object(struct drm_device *dev,
						size_t size);
int virtio_gpu_object_create(struct virtio_gpu_device *vgdev,
			     struct virtio_gpu_object_params *params,
			     struct virtio_gpu_object **bo_ptr,
			     struct virtio_gpu_fence *fence);

bool virtio_gpu_is_shmem(struct virtio_gpu_object *bo);

int virtio_gpu_resource_id_get(struct virtio_gpu_device *vgdev,
			       uint32_t *resid);

void virtio_gpu_resource_id_put(struct virtio_gpu_device *vgdev, uint32_t id);

void virtio_gpu_object_save_restore_list(struct virtio_gpu_device *vgdev,
					 struct virtio_gpu_object *bo,
					 struct virtio_gpu_object_params *params);

int virtio_gpu_object_restore_all(struct virtio_gpu_device *vgdev);

/* virtgpu_prime.c */
int virtio_gpu_resource_assign_uuid(struct virtio_gpu_device *vgdev,
				    struct virtio_gpu_object *bo);
struct dma_buf *virtgpu_gem_prime_export(struct drm_gem_object *obj,
					 int flags);
struct drm_gem_object *virtgpu_gem_prime_import(struct drm_device *dev,
						struct dma_buf *buf);
int virtgpu_gem_prime_get_uuid(struct drm_gem_object *obj,
			       uuid_t *uuid);
struct drm_gem_object *virtgpu_gem_prime_import_sg_table(
	struct drm_device *dev, struct dma_buf_attachment *attach,
	struct sg_table *sgt);

/* virtgpu_debugfs.c */
void virtio_gpu_debugfs_init(struct drm_minor *minor);
void virtio_gpu_debugfs_late_init(struct virtio_gpu_device *vgpudev);

/* virtgpu_vram.c */
bool virtio_gpu_is_vram(struct virtio_gpu_object *bo);
int virtio_gpu_vram_create(struct virtio_gpu_device *vgdev,
			   struct virtio_gpu_object_params *params,
			   struct virtio_gpu_object **bo_ptr);
struct sg_table *virtio_gpu_vram_map_dma_buf(struct virtio_gpu_object *bo,
					     struct device *dev,
					     enum dma_data_direction dir);
void virtio_gpu_vram_unmap_dma_buf(struct device *dev,
				   struct sg_table *sgt,
				   enum dma_data_direction dir);

/* virtgpu_submit.c */
int virtio_gpu_execbuffer_ioctl(struct drm_device *dev, void *data,
				struct drm_file *file);

#endif
