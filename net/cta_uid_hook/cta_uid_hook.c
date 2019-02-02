/*
 **
 **  Project: multi channels module
 **  File:      multi_channel_hook.c
 **  Author: huangjunyuan
 **  Date:    09/6/2015
 **
 **  Purpose:
 **      multi channels module.
 **
 **  History:
 **  <author>   <time>          <version >   <desc>
 */
#include <net/ip.h>
#include <linux/init.h>
#include <net/tcp.h>
#include <net/sock.h>
#include <linux/time.h>
#include <linux/init.h>
#include <net/netlink.h>
#include <linux/timer.h>
#include <linux/types.h>
#include <linux/skbuff.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/version.h>
#include <linux/proc_fs.h>
#include <linux/file.h>
#include <linux/seq_file.h>
#include <uapi/linux/rtc.h>
#include <linux/rtc.h>
#include <linux/rwlock.h>
#include <linux/rwlock_types.h>

#include <linux/pid.h>
#include <linux/export.h>
#include <linux/sched.h>
#include <linux/fs.h>
#include <linux/path.h>
#include <linux/slab.h>
#include <linux/fs_struct.h>

#include <linux/list.h>
#include <linux/icmp.h>
#include <net/icmp.h>
#include <linux/netfilter.h>
#include <linux/rtnetlink.h>
#include <linux/netfilter/xt_state.h>

#include <net/netfilter/nf_conntrack_core.h>
#include <net/netfilter/nf_conntrack_tuple.h>
#include <net/netfilter/nf_conntrack_ecache.h>
#include <linux/netfilter/nf_conntrack_common.h>

#include <asm/poll.h>
#include <asm/siginfo.h>
#include <asm/uaccess.h>

#ifdef CONFIG_SYSCTL
#include <linux/sysctl.h>
#endif

MODULE_LICENSE("GPL");
MODULE_AUTHOR("huangjunyuan0016003461@nubia.com.cn");
MODULE_DESCRIPTION("cta uid print module");

//#define CTA_KERNEL_LOW 1
#define CTA_UID_DEBUG 1
#ifdef CTA_UID_DEBUG
#define CTA_UID_TAG                 "cta_uid_print"
#define CTA_UID_PRINT(args...) \
    do{ \
        printk(CTA_UID_TAG":"args); \
    }while(0)
#else
#define CTA_UID_PRINT(args...) do{}while(0)
#endif

#define NETLINK_CTA_UID       28
#define TALKING_TO_KERNEL	0x13
#define CTA_UID_INIT              0X14
#define CTA_UID_ADD              0X15
#define APP_NAME_LEN            64

DEFINE_RWLOCK (app_info_lock);
#define app_info_read_lock()       read_lock_bh(&app_info_lock);
#define app_info_read_unlock()     read_unlock_bh(&app_info_lock);
#define app_info_write_lock()      write_lock_bh(&app_info_lock);
#define app_info_write_unlock()    write_unlock_bh(&app_info_lock);

#define IP_TO_STRING(addr) \
           ((unsigned char *)&addr)[0], \
           ((unsigned char *)&addr)[1], \
           ((unsigned char *)&addr)[2], \
           ((unsigned char *)&addr)[3]

static int cta_print_enable = 0;
static struct sock *cta_uid_sock = NULL;
static struct kmem_cache *app_info_cache __read_mostly;
static unsigned int local_ip_addr = 0x0100007f;

static char *wlan_str = "使用WLAN网络连接传送数据";
static char *rmnet_str = "使用移动通信网络数据连接传送数据";

LIST_HEAD (app_list_head);

struct app_info {
    unsigned int uid;
    char package_name[APP_NAME_LEN];
    char application_name[APP_NAME_LEN];
};

struct app_info_node {
    struct list_head list;
    unsigned int uid;
    char package_name[APP_NAME_LEN];
    char application_name[APP_NAME_LEN];
};

#ifdef CONFIG_SYSCTL
static struct ctl_table_header *cta_uid_table_header;

static struct ctl_table cta_uid_sysctl_table[] = {
    {
        .procname = "cta_print_enable",
        .data = &cta_print_enable,
        .maxlen = sizeof(int),
        .mode = 0644,
        .proc_handler = proc_dointvec,
    },
    {}
};
#endif

static void free_cta_node(void) {

    struct app_info_node *app_node = NULL;
    struct app_info_node *app_node_next = NULL;

    app_info_write_lock();
    list_for_each_entry_safe(app_node, app_node_next, &app_list_head, list)
    {
        if (app_node) {
            list_del(&app_node->list);
            kmem_cache_free(app_info_cache, app_node);
        }
    }
    app_info_write_unlock();

}

static int is_local_skb(struct iphdr *iph) {

    struct udphdr *udph;
    unsigned short dport;

    if (iph->daddr == local_ip_addr) {
        return 1;
    }
    if (iph->protocol == IPPROTO_UDP) {
        udph = (struct udphdr *) ((char *) iph + iph->ihl * 4);

#ifdef __LITTLE_ENDIAN
        dport = ntohs(udph->dest);
#else
        dport = udph->dest;
#endif
        if (dport == 53) {
            return 1;
        }
    }
    return 0;
}

static void print_app_info(unsigned int uid, unsigned int dst_ip,
        const struct net_device *out) {

    struct app_info_node *node = NULL;

    app_info_read_lock();
    list_for_each_entry(node, &app_list_head, list)
    {
        if (uid > 1000 && node->uid == uid) {
            if (!memcmp(out->name, "wlan0", strlen("wlan0"))) {
                printk("ctaifs:<%s>[Internet][%s]:[write]%s(%d.%d.%d.%d)\n",
                        node->application_name, node->package_name, wlan_str,
                        IP_TO_STRING(dst_ip));
            } else {
                printk("ctaifs:<%s>[Internet][%s]:[write]%s(%d.%d.%d.%d)\n",
                        node->application_name, node->package_name, rmnet_str,
                        IP_TO_STRING(dst_ip));
            }
        }
    }
    app_info_read_unlock();

}

#ifdef CTA_KERNEL_LOW
static unsigned int cta_uid_print(unsigned int hook,
        struct sk_buff *skb,
        const struct net_device *in,
        const struct net_device *out,
        int (*okfn)(struct sk_buff *))
#else		 
static u_int32_t cta_uid_print(void *priv, struct sk_buff *skb,
        const struct nf_hook_state *state)
#endif
        {
    struct file *filp;
    unsigned int uid;
    struct nf_conn *ct = NULL;
    enum ip_conntrack_info ctinfo;
    struct iphdr *iph = NULL;
    struct sock *sk = NULL;

    if (!cta_print_enable) {
        return NF_ACCEPT;
    }

    ct = nf_ct_get(skb, &ctinfo);
    if (NULL == ct) {
        return NF_ACCEPT;
    }

    iph = ip_hdr(skb);

    if ((IP_CT_NEW == ctinfo) && (NULL != iph)) {

        if (is_local_skb(iph)) {
            return NF_ACCEPT;
        }
        sk = skb_to_full_sk(skb);
        if (sk != NULL && skb->sk->sk_socket != NULL
                && skb->sk->sk_socket->file != NULL) {

            filp = skb->sk->sk_socket->file;
#ifdef CTA_KERNEL_LOW
            uid = filp->f_cred->fsuid;
#else
            uid = filp->f_cred->fsuid.val;
#endif
            print_app_info(uid, iph->daddr, state->out);
        }
    }

    return NF_ACCEPT;

}

static struct nf_hook_ops cta_uid_ops[] __read_mostly = {

    {
        .hook = cta_uid_print,
        .pf = PF_INET,
        .hooknum = NF_INET_LOCAL_OUT,
        .priority = NF_IP_PRI_MANGLE + 1,
    },

};

static int netlink_init_app_info(struct nlmsghdr *nlh) {

    int i = 0;
    char *p = NULL;
    int *q = NULL;
    int count = 0;
    struct app_info *tmp_node = NULL;
    struct app_info_node *node = NULL;

    q = (int *) NLMSG_DATA(nlh);
    p = (char *) NLMSG_DATA(nlh) + sizeof(int);

    count = *q;
    CTA_UID_PRINT("app count = %d\n", count);

    for (i = 0; i < count; i++) {

        node = kmem_cache_zalloc(app_info_cache, GFP_ATOMIC);

        if (NULL == node) {
            return 0;
        }
        INIT_LIST_HEAD(&node->list);

        tmp_node = (struct app_info *) p;

        node->uid = tmp_node->uid;
        memcpy(node->package_name, tmp_node->package_name, APP_NAME_LEN);
        memcpy(node->application_name, tmp_node->application_name,
                APP_NAME_LEN);

        CTA_UID_PRINT(
                "uid = %u,package name = %s, application name = %s\n", tmp_node->uid, tmp_node->package_name, tmp_node->application_name);

        app_info_write_lock();
        list_add_tail(&node->list, &app_list_head);
        app_info_write_unlock();

        p = p + sizeof(struct app_info);

    }
    return 0;
}

static int __cta_uid_rcv_msg(struct sk_buff *skb, struct nlmsghdr *nlh) {
    int ret = 0;
    switch (nlh->nlmsg_type) {
    case TALKING_TO_KERNEL:
        CTA_UID_PRINT("now kernel rcv ap netlink app pid\n");
        break;

    case CTA_UID_INIT:
    case CTA_UID_ADD:
        ret = netlink_init_app_info(nlh);
        break;

    default:
        break;
    }

    return ret;

}

static void cta_uid_netlink_rcv(struct sk_buff *skb) {
    netlink_rcv_skb(skb, &__cta_uid_rcv_msg);
}

static int cta_uid_netlink_init(void) {
struct netlink_kernel_cfg cfg = {
    .input = cta_uid_netlink_rcv
};

cta_uid_sock = netlink_kernel_create(&init_net, NETLINK_CTA_UID,&cfg);

if (NULL == cta_uid_sock) {
    printk("Create cta_uid_sock sock failed\n");
    return -1;
}
return 0;
}

static void cta_uid_netlink_exit(void) {
if (cta_uid_sock) {
    netlink_kernel_release(cta_uid_sock);
}
return;
}

static void __exit cta_uid_fini(void)
{
cta_print_enable = 0;
free_cta_node();
kmem_cache_destroy(app_info_cache);

#ifdef CONFIG_SYSCTL
unregister_net_sysctl_table(cta_uid_table_header);
#endif

cta_uid_netlink_exit();
nf_unregister_hooks(cta_uid_ops, ARRAY_SIZE(cta_uid_ops));
}

static int __init cta_uid_init(void)
{

app_info_cache= kmem_cache_create("app_info_cache",
        sizeof(struct app_info_node),
        0,
        SLAB_HWCACHE_ALIGN, NULL);
if (!app_info_cache) {
    return -ENOMEM;
}

#ifdef CONFIG_SYSCTL
cta_uid_table_header= register_net_sysctl(&init_net, "net/cta_uid_print", cta_uid_sysctl_table);
if (cta_uid_table_header == NULL) {
    return -ENOMEM;
}
#endif
nf_register_hooks(cta_uid_ops, ARRAY_SIZE(cta_uid_ops));
cta_uid_netlink_init();

return 0;
}

module_init (cta_uid_init);
module_exit (cta_uid_fini);
