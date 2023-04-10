#include "arm_math.h" /* FUTs */
#include "arr_desc.h"
#include "basic_math_templates.h"
#include "basic_math_test_data.h"
#include "jtest.h"
#include "ref.h" /* Reference Functions */
#include "test_templates.h"
#include "type_abbrev.h"

#define JTEST_ARM_SUB_TEST(suffix)                                     \
  BASIC_MATH_DEFINE_TEST_TEMPLATE_BUF2_BLK(                            \
      sub, suffix, TYPE_FROM_ABBREV(suffix), TYPE_FROM_ABBREV(suffix), \
      BASIC_MATH_COMPARE_INTERFACE)

JTEST_ARM_SUB_TEST(f32);
JTEST_ARM_SUB_TEST(q31);
JTEST_ARM_SUB_TEST(q15);
JTEST_ARM_SUB_TEST(q7);

/*--------------------------------------------------------------------------------*/
/* Collect all tests in a group. */
/*--------------------------------------------------------------------------------*/

JTEST_DEFINE_GROUP(sub_tests) {
  JTEST_TEST_CALL(arm_sub_f32_test);
  JTEST_TEST_CALL(arm_sub_q31_test);
  JTEST_TEST_CALL(arm_sub_q15_test);
  JTEST_TEST_CALL(arm_sub_q7_test);
}
