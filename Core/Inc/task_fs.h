/**
  ******************************************************************************
  * @file    task_fs.h
  * @brief   Filesystem task handlers header file
  * @note    Contains handlers for filesystem-related tasks
  ******************************************************************************
  */
#ifndef INC_TASK_FS_H_
#define INC_TASK_FS_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Forward declarations ------------------------------------------------------*/
typedef struct task_data_t task_data_t;

/* Exported function prototypes ----------------------------------------------*/
/* Shell Commands */
void handle_fs_mount(const task_data_t *task_data);
void handle_fs_unmount(const task_data_t *task_data);
void handle_fs_df(const task_data_t *task_data);
void handle_fs_ls(const task_data_t *task_data);
void handle_fs_cat(const task_data_t *task_data);
void handle_fs_write(const task_data_t *task_data);
void handle_fs_rm(const task_data_t *task_data);
void handle_fs_mkdir(const task_data_t *task_data);
void handle_fs_rmdir(const task_data_t *task_data);
void handle_fs_cp(const task_data_t *task_data);

/* Scheduling function prototypes ---------------------------------------------*/
/* Shell Commands */
void schedule_fs_mount(void);
void schedule_fs_unmount(void);
void schedule_fs_df(void);
void schedule_fs_ls(void);
void schedule_fs_cat(void);
void schedule_fs_write(void);
void schedule_fs_rm(void);
void schedule_fs_mkdir(void);
void schedule_fs_rmdir(void);
void schedule_fs_cp(void);

#ifdef __cplusplus
}
#endif
#endif /* INC_TASK_FS_H_ */