# include <linux/slab.h>
# include <linux/string.h>

#include <linux/sort.h>


void
qsort(void *const pbase, int total_elems, int size,
 int (*cmp)(const void*, const void*))
{
    sort(pbase,total_elems, size, cmp, NULL);
}
