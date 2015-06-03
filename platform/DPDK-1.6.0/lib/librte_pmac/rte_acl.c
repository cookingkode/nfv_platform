/*-
 *      Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
 *      All rights reserved.
 *   
 *      Redistribution and use in source and binary forms, with or without
 *      modification, are permitted provided that the following conditions
 *      are met:
 *   
 *        * Redistributions of source code must retain the above copyright
 *          notice, this list of conditions and the following disclaimer.
 *        * Redistributions in binary form must reproduce the above copyright
 *          notice, this list of conditions and the following disclaimer in
 *          the documentation and/or other materials provided with the
 *          distribution.
 *        * Neither the name of Intel Corporation nor the names of its
 *          contributors may be used to endorse or promote products derived
 *          from this software without specific prior written permission.
 *  
 *        * The use of this source code in binary form is permitted to only
 *          be used on Intel� Architecture Processors.
 *   
 *      THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *      AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *      IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *      ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 *      LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *      DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *      SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *      CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *      OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *      USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 */

#include <rte_acl.h>
#include "acl.h"

#define	BIT_SIZEOF(x)	(sizeof(x) * CHAR_BIT)

TAILQ_HEAD(rte_acl_list, rte_acl_ctx);

struct rte_acl_ctx *
rte_acl_find_existing(const char *name)
{
	struct rte_acl_ctx *ctx;
	struct rte_acl_list *acl_list;

	/* check that we have an initialised tail queue */
	if ((acl_list = RTE_TAILQ_LOOKUP_BY_IDX(RTE_TAILQ_ACL,
			rte_acl_list)) == NULL) {
		rte_errno = E_RTE_NO_TAILQ;
		return NULL;
	}

	rte_rwlock_read_lock(RTE_EAL_TAILQ_RWLOCK);
	TAILQ_FOREACH(ctx, acl_list, next) {
		if (strncmp(name, ctx->name, sizeof(ctx->name)) == 0)
			break;
	}
	rte_rwlock_read_unlock(RTE_EAL_TAILQ_RWLOCK);

	if (ctx == NULL)
		rte_errno = ENOENT;
	return (ctx);
}

void
rte_acl_free(struct rte_acl_ctx *ctx)
{
	if (ctx == NULL)
		return;

	RTE_EAL_TAILQ_REMOVE(RTE_TAILQ_ACL, rte_acl_list, ctx);

	rte_free(ctx->mem);
	rte_free(ctx);
}

struct rte_acl_ctx *
rte_acl_create(const struct rte_acl_param *param)
{
	size_t sz;
	struct rte_acl_ctx *ctx;
	struct rte_acl_list *acl_list;
	char name[sizeof (ctx->name)];

	/* check that we have an initialised tail queue */
	if ((acl_list = RTE_TAILQ_LOOKUP_BY_IDX(RTE_TAILQ_ACL,
			rte_acl_list)) == NULL) {
		rte_errno = E_RTE_NO_TAILQ;
		return NULL;
	}

	/* check that input parameters are valid. */
	if (param == NULL || param->name == NULL) {
		rte_errno = EINVAL;
		return (NULL);
	}

	rte_snprintf(name, sizeof(name), "ACL_%s", param->name);

	/* calculate amount of memory required for pattern set. */
	sz = sizeof (*ctx) + param->max_rule_num * param->rule_size;

	/* add additional space for build_rule vs rule */
	sz += param->max_rule_num * (sizeof(struct rte_acl_build_rule) -
		sizeof(struct rte_acl_rule));

	/* get EAL TAILQ lock. */
	rte_rwlock_write_lock(RTE_EAL_TAILQ_RWLOCK);

	/* if we already have one with that name */
	TAILQ_FOREACH(ctx, acl_list, next) {
		if (strncmp(param->name, ctx->name, sizeof (ctx->name)) == 0)
			break;
	}

	/* if ACL with such name doesn't exist, then create a new one. */
	if (ctx == NULL && (ctx = rte_zmalloc_socket(name, sz, CACHE_LINE_SIZE,
			param->socket_id)) != NULL) {

		/* init new allocated context. */
		ctx->rules = ctx + 1;
		ctx->max_rules = param->max_rule_num;
		ctx->rule_sz = param->rule_size;
		ctx->socket_id = param->socket_id;
		rte_snprintf(ctx->name, sizeof(ctx->name), "%s", param->name);

		TAILQ_INSERT_TAIL(acl_list, ctx, next);

	} else if (ctx == NULL) {
		RTE_LOG(ERR, PMAC,
			"allocation of %zu bytes on socket %d for %s failed\n",
			sz, param->socket_id, name);
	}
	ctx->build_rules = ctx->rules;
	ctx->fields = (struct rte_acl_field *)(ctx->build_rules + ctx->max_rules);
	rte_rwlock_write_unlock(RTE_EAL_TAILQ_RWLOCK);
	return (ctx);
}

static int
acl_add_rules(struct rte_acl_ctx *ctx, const void *rules, uint32_t num)
{
	uint8_t *pos;

	if (num + ctx->num_rules > ctx->max_rules)
		return (-ENOMEM);

	pos = ctx->rules;
	pos += ctx->rule_sz * ctx->num_rules;
	memcpy(pos, rules, num * ctx->rule_sz);
	ctx->num_rules += num;

	return (0);
}

static inline int
rte_acl_check_rule(const struct rte_acl_rule_data *rd)
{
	if ((rd->category_mask & LEN2MASK(RTE_ACL_MAX_CATEGORIES)) == 0 ||
			rd->priority > RTE_ACL_MAX_PRIORITY ||
			rd->priority < RTE_ACL_MIN_PRIORITY ||
			rd->userdata == RTE_ACL_INVALID_USERDATA)
		return (-EINVAL);
	return (0);
}

static inline int
rte_acl_ipv4vlan_check_rule(const struct rte_acl_ipv4vlan_rule *rule)
{
	if (rule->src_port_low > rule->src_port_high ||
			rule->dst_port_low > rule->dst_port_high ||
			rule->src_mask_len > BIT_SIZEOF(rule->src_addr) ||
			rule->dst_mask_len > BIT_SIZEOF(rule->dst_addr))
		return (-EINVAL);

	return (rte_acl_check_rule(&rule->data));
}

void
rte_acl_ctx_rules_reset(struct rte_acl_ctx *ctx) 
{
	//size_t sz;

	if (ctx) {/*
		sz = ctx->max_rules * ctx->rule_sz;
		sz += ctx->max_rules * (sizeof(struct rte_acl_build_rule) -
			sizeof(struct rte_acl_rule));
		ctx->rules = ctx + 1;

		memset(ctx->rules, 0, sz);

		ctx->build_rules = ctx->rules;
		ctx->fields = (struct rte_acl_field *)(ctx->build_rules + ctx->max_rules);*/
		ctx->num_rules = 0;
    }

	return;
}

int
rte_acl_add_rules(struct rte_acl_ctx *ctx,
	const struct rte_acl_rule *rules,
	uint32_t num)
{
	uint32_t n;
	struct rte_acl_build_rule *new_rule;
	struct rte_acl_field *fields;

	if (ctx == NULL || rules == NULL || 0 == ctx->rule_sz ||
		ctx->config.num_fields == 0)
		return (-EINVAL);
	if (num + ctx->num_rules > ctx->max_rules)
		return (-ENOMEM);

	new_rule = ctx->build_rules + ctx->num_rules;
	fields = ctx->fields + (ctx->num_rules * ctx->config.num_fields);

	for (n=0; n<num; n++) {
		new_rule->next = NULL;
		if (ctx->num_rules != 0)
			(new_rule - 1)->next = new_rule;
		new_rule->config = &ctx->config;
		memcpy(&new_rule->f, rules, sizeof(struct rte_acl_rule));
		memcpy(fields, rules->field, ctx->config.num_fields * 
			sizeof(struct rte_acl_field));
		new_rule->f.field = fields;
		ctx->num_rules++;
		rules++;
		fields += ctx->config.num_fields;
		new_rule++;
	}

	return (0);
}

int
rte_acl_set_config(struct rte_acl_ctx *ctx,
	const struct rte_acl_config *cfg)
{
	if (cfg->num_fields >= RTE_ACL_MAX_FIELDS)
		return -(EINVAL);
	ctx->config.num_fields = cfg->num_fields;
	memcpy(ctx->config.defs, cfg->defs, 
		cfg->num_fields * sizeof(cfg->defs[0]));
	return 0;
}

int
rte_acl_ipv4vlan_add_rules(struct rte_acl_ctx *ctx,
	const struct rte_acl_ipv4vlan_rule *rules,
	uint32_t num)
{
	int rc;
	uint32_t i;

	if (ctx == NULL || rules == NULL || sizeof (rules[0]) != ctx->rule_sz)
		return -(EINVAL);

	for (i = 0; i != num; i++) {
		if ((rc = rte_acl_ipv4vlan_check_rule(rules + i)) != 0) {
			RTE_LOG(ERR, PMAC, "%s(%s): rule #%u is invalid\n",
				__func__, ctx->name, i + 1);
			return (rc);
		}
	}

	return (acl_add_rules(ctx, rules, num));
}

/*
 * Dump ACL context to the stdout.
 */
void
rte_acl_dump(const struct rte_acl_ctx *ctx)
{
	if (!ctx)
		return;
	printf("acl context <%s>@%p\n", ctx->name, ctx);
	printf("  max_rules=%"PRIu32"\n", ctx->max_rules);
	printf("  rule_size=%"PRIu32"\n", ctx->rule_sz);
	printf("  num_rules=%"PRIu32"\n", ctx->num_rules);
	printf("  num_categories=%"PRIu32"\n", ctx->num_categories);
	printf("  num_tries=%"PRIu32"\n", ctx->num_tries);
}

/*
 * Dump all ACL contexts to the stdout.
 */
void
rte_acl_list_dump(void)
{
	struct rte_acl_ctx *ctx;
	struct rte_acl_list *acl_list;

	/* check that we have an initialised tail queue */
	if ((acl_list = RTE_TAILQ_LOOKUP_BY_IDX(RTE_TAILQ_ACL,
			rte_acl_list)) == NULL) {
		rte_errno = E_RTE_NO_TAILQ;
		return;
	}

	rte_rwlock_read_lock(RTE_EAL_TAILQ_RWLOCK);
	TAILQ_FOREACH(ctx, acl_list, next) {
		rte_acl_dump(ctx);
	}
	rte_rwlock_read_unlock(RTE_EAL_TAILQ_RWLOCK);
}
