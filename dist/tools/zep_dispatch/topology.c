/*
 * Copyright (C) 2021 Benjamin Valentin
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License v2. See the file LICENSE for more details.
 */

#include <arpa/inet.h>
#include <netdb.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>

#include "kernel_defines.h"
#include "topology.h"
#include "zep_parser.h"

#define NODE_NAME_MAX_LEN   32
#define HW_ADDR_MAX_LEN      8

struct node {
    list_node_t next;
    char name[NODE_NAME_MAX_LEN];
    uint8_t mac[HW_ADDR_MAX_LEN];
    struct sockaddr_in6 addr;
    uint32_t num_tx;
    uint32_t num_rx;
    uint8_t mac_len;
};

struct edge {
    list_node_t next;
    struct node *a;
    struct node *b;
    float weight_a_b;
    float weight_b_a;
};

size_t l2util_addr_from_str(const char *str, uint8_t *out);

static char *_fmt_addr(char *out, size_t out_len, const uint8_t *addr, uint8_t addr_len)
{
    char *start = out;

    if (out_len < 3 * addr_len) {
        return NULL;
    }

    while (addr_len--) {
        out += sprintf(out, "%02X", *addr++);
        *(out++) = addr_len ? ':' : '\0';
    }

    return start;
}

static struct node *_find_node_by_name(const list_node_t *nodes, const char *name)
{
    for (list_node_t *node = nodes->next; node; node = node->next) {
        struct node *super = container_of(node, struct node, next);
        if (strncmp(super->name, name, sizeof(super->name)) == 0) {
            return super;
        }
    }

    return NULL;
}

static struct node *_find_or_create_node(list_node_t *nodes, const char *name)
{
    struct node *node = _find_node_by_name(nodes, name);

    if (node == NULL) {
        node = calloc(1, sizeof(*node));
        strncpy(node->name, name, sizeof(node->name) - 1);
        list_add(nodes, &node->next);
    }

    return node;
}

static bool _parse_line(char *line, list_node_t *nodes, list_node_t *edges)
{
    struct edge *e;

    if (*line == '#') {
        return true;
    }

    char *a     = strtok(line, "\n\t ");
    char *b     = strtok(NULL, "\n\t ");
    char *e_ab  = strtok(NULL, "\n\t ");
    char *e_ba  = strtok(NULL, "\n\t ");

    if (a == NULL) {
        return false;
    }

    if (b == NULL) {
        _find_or_create_node(nodes, a);
        return true;
    }

    /* add node with a defined MAC address */
    if (strcmp(b, ":=") == 0) {
        struct node *n = _find_or_create_node(nodes, a);
        if (n == NULL) {
            return false;
        }

        n->mac_len = l2util_addr_from_str(e_ab, n->mac);
        return true;
    }

    if (e_ab == NULL) {
        e_ab = "1";
    }

    if (e_ba == NULL) {
        e_ba = e_ab;
    }

    e = malloc(sizeof(*e));

    e->a = _find_or_create_node(nodes, a);
    e->b = _find_or_create_node(nodes, b);
    e->weight_a_b = atof(e_ab);
    e->weight_b_a = atof(e_ba);

    list_add(edges, &e->next);

    return true;
}

int topology_print(const char *file, const topology_t *t)
{
    FILE *out;
    char addr_str[3 * HW_ADDR_MAX_LEN];

    if (t->flat) {
        // TODO
        return 0;
    }

    if (strcmp(file, "-") == 0) {
        out = stdout;
    }
    else {
        out = fopen(file, "w");
    }

    if (out == NULL) {
        return -1;
    }

    fprintf(out, "digraph G {\n");

    for (list_node_t *node = t->nodes.next; node; node = node->next) {
        struct node *super = container_of(node, struct node, next);
        fprintf(out, "\t%s [ label = \"%s\\n[%s]\" ]\n",
                super->name, super->name,
                super->mac_len ? _fmt_addr(addr_str, sizeof(addr_str), super->mac, super->mac_len)
                               : "disconnected");
    }

    fprintf(out, "\n");

    for (list_node_t *edge = t->edges.next; edge; edge = edge->next) {
        struct edge *super = container_of(edge, struct edge, next);
        if (super->weight_a_b) {
            fprintf(out, "\t%s -> %s [ label = \"%.2f\" ]\n",
                    super->a->name, super->b->name, super->weight_a_b);
        }
        if (super->weight_b_a) {
            fprintf(out, "\t%s -> %s [ label = \"%.2f\" ]\n",
                    super->b->name, super->a->name, super->weight_b_a);
        }
    }

    fprintf(out, "}\n");

    if (out != stdout) {
        fclose(out);
    }

    return 0;
}

void topology_print_stats(const topology_t *t, bool reset)
{
    uint32_t tx_total = 0;

    puts("{ nodes: [");
    for (list_node_t *node = t->nodes.next; node; node = node->next) {
        struct node *super = container_of(node, struct node, next);

        tx_total += super->num_tx;

        printf("\t{ name: %s, tx: %u, rx: %u }%c\n",
               super->name, super->num_tx, super->num_rx, node->next ? ',' : ' ');
        if (reset) {
            super->num_tx = 0;
            super->num_rx = 0;
        }
    }
    printf("], tx_total: %u }\n", tx_total);
}

int topology_parse(const char *file, topology_t *out)
{
    FILE *in;

    memset(out, 0, sizeof(*out));

    if (strcmp(file, "-") == 0) {
        in = stdin;
    }
    else {
        in = fopen(file, "r");
    }

    if (in == NULL) {
        return -1;
    }

    char *line = NULL;
    size_t line_len = 0;

    while (getline(&line, &line_len, in) > 0) {
        _parse_line(line, &out->nodes, &out->edges);
    }

    if (line) {
        free(line);
    }

    return 0;
}

void topology_send(const topology_t *t, int sock,
                   const struct sockaddr_in6 *src_addr,
                   void *buffer, size_t len)
{
    struct node *sender = NULL;

    if (t->has_sniffer) {
        sendto(sock, buffer, len, 0,
               (struct sockaddr *)&t->sniffer_addr, sizeof(t->sniffer_addr));
    }

    for (list_node_t *edge = t->edges.next; edge; edge = edge->next) {
        struct edge *super = container_of(edge, struct edge, next);

        if (!super->a->mac_len || !super->b->mac_len) {
            continue;
        }

        if (memcmp(&super->a->addr, src_addr, sizeof(*src_addr)) == 0) {
            if (sender == NULL) {
                sender = super->a;
                sender->num_tx++;
            }

            /* packet loss */
            if (random() > super->weight_a_b * RAND_MAX) {
                continue;
            }
            zep_set_lqi(buffer, super->weight_a_b * 0xFF);
            sendto(sock, buffer, len, 0,
                   (struct sockaddr *)&super->b->addr,
                   sizeof(super->b->addr));
            super->b->num_rx++;
        }
        else if (memcmp(&super->b->addr, src_addr, sizeof(*src_addr)) == 0) {
            if (sender == NULL) {
                sender = super->b;
                sender->num_tx++;
            }

            /* packet loss */
            if (random() > super->weight_b_a * RAND_MAX) {
                continue;
            }
            zep_set_lqi(buffer, super->weight_b_a * 0xFF);
            sendto(sock, buffer, len, 0,
                   (struct sockaddr *)&super->a->addr,
                   sizeof(super->a->addr));
            super->a->num_rx++;
        }
    }
}

bool topology_add(topology_t *t, const uint8_t *mac, uint8_t mac_len,
                  struct sockaddr_in6 *addr)
{
    struct node *empty = NULL;
    char addr_str[3 * HW_ADDR_MAX_LEN];

    if (mac_len > HW_ADDR_MAX_LEN) {
        fprintf(stderr, "discarding frame with %u byte address\n", mac_len);
        return false;
    }

    for (list_node_t *node = t->nodes.next; node; node = node->next) {
        struct node *super = container_of(node, struct node, next);

        /* store free node */
        if (!super->mac_len) {
            empty = super;
            continue;
        }

        if (mac_len != super->mac_len) {
            continue;
        }

        /* node is already in the list - either it is connected or MAC was pinned */
        if (memcmp(super->mac, mac, mac_len) == 0) {
            if (super->addr.sin6_port == addr->sin6_port) {
                /* abort if node is already connected */
                return true;
            } else {
                /* use pre-allocated node */
                empty = super;
                break;
            }
        }
    }

    /* topology full - can't add node */
    if (empty == NULL) {
        fprintf(stderr, "can't add %s - topology full\n",
                _fmt_addr(addr_str, sizeof(addr_str), mac, mac_len));
        return false;
    }

    printf("adding node %s as %s\n",
            _fmt_addr(addr_str, sizeof(addr_str), mac, mac_len),
            (char *)empty->name);

    /* add new node to empty spot */
    memcpy(empty->mac, mac, sizeof(empty->mac));
    memcpy(&empty->addr, addr, sizeof(empty->addr));
    empty->mac_len = mac_len;

    return true;
}

void topology_set_sniffer(topology_t *t, struct sockaddr_in6 *addr)
{
    char addr_str[INET6_ADDRSTRLEN];
    getnameinfo((struct sockaddr*)addr, sizeof(*addr),
                addr_str, sizeof(addr_str), 0, 0, NI_NUMERICHOST);
    if (t->has_sniffer) {
        printf("replace sniffer with %s\n", addr_str);
    } else {
        printf("adding sniffer %s\n", addr_str);
    }

    memcpy(&t->sniffer_addr, addr, sizeof(t->sniffer_addr));
    t->has_sniffer = true;
}
