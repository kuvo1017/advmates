/* header file for autogl_callback.c */

#ifndef __AUTOGL_CALLBACK_INSIDE__
#define __AUTOGL_CALLBACK_INSIDE__



int AutoGL_INSIDE_IsMaster (void);

void AutoGL_INSIDE_InvokeCallback (AutoGL_Property *property);

void AutoGL_INSIDE_InvokeViewRedrawCallback (void);

void AutoGL_INSIDE_InvokeViewEventCallback (void);

void AutoGL_INSIDE_InvokeIdleEventCallback (void);



#endif  /* __AUTOGL_CALLBACK_INSIDE__ */
