/***************************************************************************
 *                      COPYRIGHT NOTICE
 *             Copyright 2020 Horizon Robotics, Inc.
 *                     All rights reserved.
 ***************************************************************************/

/**
 * @file cam_diag_list.h
 *
 * @NO{S10E02C07}
 * @ASIL{B}
 */

#ifndef INC_CAM_DIAG_LIST_H_
#define INC_CAM_DIAG_LIST_H_
#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
#include <stdlib.h>

struct list_node {
	struct list_node *next, *prev;
	uint32_t count;
};

void diag_init_list_head(struct list_node *list);
void diag_list_add(struct list_node *new_node, struct list_node *head);
void diag_list_add_tail(struct list_node *new_node, struct list_node *head);
void diag_list_add_before(struct list_node *new_node, struct list_node *head);
void diag_list_add_prev(struct list_node *new_node, struct list_node *head);
void diag_list_del(struct list_node *entry);
int diag_list_is_last(const struct list_node *list,
	const struct list_node *head);
int diag_list_empty(const struct list_node *head);

#define list_first(head) ((head)->next)
#define list_last(head) ((head)->prev)

#define list_for_each(pos, head) \
	for (pos = (head)->next; pos != (head); pos = pos->next)

#define list_for_each_safe(pos, n, head) \
	for (pos = (head)->next, n = pos->next; pos != (head); \
		pos = n, n = pos->next)

#define list_for_each_diag_node(pos, n, first) \
	for (pos = first, n = pos->node_next; pos != NULL; \
		pos = n, n = pos->node_next)
#ifdef __cplusplus
}
#endif
#endif  // INC_CAM_DIAG_LIST_H_
