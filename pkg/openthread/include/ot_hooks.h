#ifndef OT_H
#define OT_H

#ifdef __cplusplus
extern "C" {
#endif

netdev_t* openthread_get_netdev(void);
otInstance* openthread_get_instance(void); 
#ifdef __cplusplus
}
#endif

#endif /* OT_H */
/** @} */
