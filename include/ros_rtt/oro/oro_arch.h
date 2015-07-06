
#ifndef __ORO_ARCH_x86_64__
#define __ORO_ARCH_x86_64__

#ifndef CONFIG_FORCE_UP
#define ORO_LOCK "lock ; "
#else
#define ORO_LOCK ""
#endif

typedef struct { volatile int counter; } oro_atomic_t;

#define ORO_ATOMIC_SETUP	oro_atomic_set
#define ORO_ATOMIC_CLEANUP(v)

#define oro_atomic_read(v)		((v)->counter)

#define oro_atomic_set(v,i)		(((v)->counter) = (i))

static __inline__ void oro_atomic_add(oro_atomic_t *v, int i)
{
	__asm__ __volatile__(
		ORO_LOCK "addl %1,%0"
		:"=m" (v->counter)
		:"ir" (i), "m" (v->counter));
}

static __inline__ void oro_atomic_sub(oro_atomic_t *v, int i)
{
	__asm__ __volatile__(
		ORO_LOCK "subl %1,%0"
		:"=m" (v->counter)
		:"ir" (i), "m" (v->counter));
}

static __inline__ int oro_atomic_sub_and_test(oro_atomic_t *v, int i)
{
	unsigned char c;

	__asm__ __volatile__(
		ORO_LOCK "subl %2,%0; sete %1"
		:"=m" (v->counter), "=qm" (c)
		:"ir" (i), "m" (v->counter) : "memory");
	return c;
}

static __inline__ void oro_atomic_inc(oro_atomic_t *v)
{
	__asm__ __volatile__(
		ORO_LOCK "incl %0"
		:"=m" (v->counter)
		:"m" (v->counter));
}

static __inline__ void oro_atomic_dec(oro_atomic_t *v)
{
	__asm__ __volatile__(
		ORO_LOCK "decl %0"
		:"=m" (v->counter)
		:"m" (v->counter));
}

static __inline__ int oro_atomic_dec_and_test(oro_atomic_t *v)
{
	unsigned char c;

	__asm__ __volatile__(
		ORO_LOCK "decl %0; sete %1"
		:"=m" (v->counter), "=qm" (c)
		:"m" (v->counter) : "memory");
	return c != 0;
}

static __inline__ int oro_atomic_inc_and_test(oro_atomic_t *v)
{
	unsigned char c;

	__asm__ __volatile__(
		ORO_LOCK "incl %0; sete %1"
		:"=m" (v->counter), "=qm" (c)
		:"m" (v->counter) : "memory");
	return c != 0;
}

static __inline__ int oro_atomic_add_negative(int i, oro_atomic_t *v)
{
	unsigned char c;

	__asm__ __volatile__(
		ORO_LOCK "addl %2,%0; sets %1"
		:"=m" (v->counter), "=qm" (c)
		:"ir" (i), "m" (v->counter) : "memory");
	return c;
}

#ifndef CONFIG_FORCE_UP
#define ORO_LOCK_PREFIX "lock ; "
#else
#define ORO_LOCK_PREFIX ""
#endif

struct oro__xchg_dummy { unsigned long a[100]; };
#define oro__xg(x) ((struct oro__xchg_dummy *)(x))

static inline unsigned long __oro_cmpxchg(volatile void *ptr, unsigned long old,
                      unsigned long _new, int size)
{
    unsigned long prev;
    switch (size) {
    case 1:
        __asm__ __volatile__(ORO_LOCK_PREFIX "cmpxchgb %b1,%2"
                     : "=a"(prev)
                     : "q"(_new), "m"(*oro__xg(ptr)), "0"(old)
                     : "memory");
        return prev;
    case 2:
        __asm__ __volatile__(ORO_LOCK_PREFIX "cmpxchgw %w1,%2"
                     : "=a"(prev)
                     : "q"(_new), "m"(*oro__xg(ptr)), "0"(old)
                     : "memory");
        return prev;
    case 4:
        __asm__ __volatile__(ORO_LOCK_PREFIX "cmpxchgl %k1,%2"
                     : "=a"(prev)
                     : "q"(_new), "m"(*oro__xg(ptr)), "0"(old)
                     : "memory");
        return prev;
    case 8:
        __asm__ __volatile__(ORO_LOCK_PREFIX "cmpxchgq %1,%2"
                     : "=a"(prev)
                     : "q"(_new), "m"(*oro__xg(ptr)), "0"(old)
                     : "memory");
        return prev;

    }
    return old;
}

#define oro_cmpxchg(ptr,o,n)\
    ((__typeof__(*(ptr)))__oro_cmpxchg((ptr),(unsigned long)(o),\
                    (unsigned long)(n),sizeof(*(ptr))))

#undef ORO_LOCK_PREFIX
#undef ORO_LOCK
#endif
