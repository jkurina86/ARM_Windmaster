/**
  ******************************************************************************
  * @file    task_fs.h
  * @brief   Filesystem task handlers header file
  * @note    Contains handlers for filesystem-related tasks.
  *          All handlers get paths from filesystem_get_buffers() instead of args.
  ******************************************************************************
  */
#ifndef INC_TASK_FS_H_
#define INC_TASK_FS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Exported function prototypes ----------------------------------------------*/
/* All handlers take const void *arg (unused - paths come from filesystem buffers) */
void handle_fs_mount(const void *arg);
void handle_fs_unmount(const void *arg);
void handle_fs_df(const void *arg);
void handle_fs_ls(const void *arg);
void handle_fs_cat(const void *arg);
void handle_fs_write(const void *arg);
void handle_fs_rm(const void *arg);
void handle_fs_mkdir(const void *arg);
void handle_fs_rmdir(const void *arg);
void handle_fs_cp(const void *arg);

#ifdef __cplusplus
}
#endif
#endif /* INC_TASK_FS_H_ */