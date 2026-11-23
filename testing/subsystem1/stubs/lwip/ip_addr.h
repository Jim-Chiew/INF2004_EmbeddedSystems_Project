typedef struct {int a;} ip_addr_t;
static inline int ipaddr_aton(const char*s, ip_addr_t*a){(void)s;(void)a;return 1;}