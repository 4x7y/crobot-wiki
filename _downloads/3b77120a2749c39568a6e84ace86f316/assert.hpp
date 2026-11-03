// clang-format off
#if !defined(CROBOT_ASSERT_ENABLED)
  #if !defined(NDEBUG) // if we are in debug mode
    #define CROBOT_ASSERT_ENABLED 1 // enable them
  #endif
#endif

#if !defined(CROBOT_ASSERT_DEFAULT_LEVEL)
  #define CROBOT_ASSERT_DEFAULT_LEVEL Debug
#endif

// -- implementation -----------------------------------------------------------

#if (defined(__GNUC__) && ((__GNUC__ * 1000 + __GNUC_MINOR__ * 100) >= 4600)) || defined(__clang__)
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wvariadic-macros"
#endif

#if defined(__clang__)
  #pragma GCC diagnostic ignored "-Wgnu-zero-variadic-macro-arguments"
#endif

#if !defined(CROBOT_ASSERT_H)
  #define CROBOT_ASSERT_H

  #define CROBOT_ASSERT(...)                    CROBOT_ASSERT_(crobot::common::implementation::AssertLevel::CROBOT_ASSERT_DEFAULT_LEVEL, __VA_ARGS__)
  #define CROBOT_ASSERT_WARNING(...)            CROBOT_ASSERT_(crobot::common::implementation::AssertLevel::Warning, __VA_ARGS__)
  #define CROBOT_ASSERT_DEBUG(...)              CROBOT_ASSERT_(crobot::common::implementation::AssertLevel::Debug, __VA_ARGS__)
  #define CROBOT_ASSERT_ERROR(...)              CROBOT_ASSERT_(crobot::common::implementation::AssertLevel::Error, __VA_ARGS__)
  #define CROBOT_ASSERT_FATAL(...)              CROBOT_ASSERT_(crobot::common::implementation::AssertLevel::Fatal, __VA_ARGS__)
  #define CROBOT_ASSERT_CUSTOM(level, ...)      CROBOT_ASSERT_(level, __VA_ARGS__)

  #define CROBOT_ASSERT_USED(...)               CROBOT_ASSERT_USED_(__VA_ARGS__)
  #define CROBOT_ASSERT_USED_WARNING(...)       CROBOT_ASSERT_USED_(crobot::common::implementation::AssertLevel::Warning, __VA_ARGS__)
  #define CROBOT_ASSERT_USED_DEBUG(...)         CROBOT_ASSERT_USED_(crobot::common::implementation::AssertLevel::Debug, __VA_ARGS__)
  #define CROBOT_ASSERT_USED_ERROR(...)         CROBOT_ASSERT_USED_(crobot::common::implementation::AssertLevel::Error, __VA_ARGS__)
  #define CROBOT_ASSERT_USED_FATAL(...)         CROBOT_ASSERT_USED_(crobot::common::implementation::AssertLevel::Fatal, __VA_ARGS__)
  #define CROBOT_ASSERT_USED_CUSTOM(level, ...) CROBOT_ASSERT_USED_(level, __VA_ARGS__)


  #define CROBOT_ASSERT_JOIN(lhs, rhs)   CROBOT_ASSERT_JOIN_(lhs, rhs)
  #define CROBOT_ASSERT_JOIN_(lhs, rhs)  CROBOT_ASSERT_JOIN__(lhs, rhs)
  #define CROBOT_ASSERT_JOIN__(lhs, rhs) lhs##rhs

  #define CROBOT_ASSERT_FILE __FILE__
  #define CROBOT_ASSERT_LINE __LINE__
  #if defined(__GNUC__) || defined(__clang__)
    #define CROBOT_ASSERT_FUNCTION __PRETTY_FUNCTION__
  #elif defined(_MSC_VER)
    #define CROBOT_ASSERT_FUNCTION __FUNCSIG__
  #elif defined(__SUNPRO_CC)
    #define CROBOT_ASSERT_FUNCTION __func__
  #else
    #define CROBOT_ASSERT_FUNCTION __FUNCTION__
  #endif

  #if defined(_MSC_VER)
    #define CROBOT_ASSERT_ALWAYS_INLINE __forceinline
  #elif defined(__GNUC__) || defined(__clang__)
    #define CROBOT_ASSERT_ALWAYS_INLINE inline __attribute__((always_inline))
  #else
    #define CROBOT_ASSERT_ALWAYS_INLINE inline
  #endif

  #define CROBOT_ASSERT_NO_MACRO

  #define CROBOT_ASSERT_APPLY_VA_ARGS(M, ...) CROBOT_ASSERT_APPLY_VA_ARGS_(M, (__VA_ARGS__))
  #define CROBOT_ASSERT_APPLY_VA_ARGS_(M, args) M args

  #define CROBOT_ASSERT_NARG(...) CROBOT_ASSERT_APPLY_VA_ARGS(CROBOT_ASSERT_NARG_, CROBOT_ASSERT_NO_MACRO,##__VA_ARGS__,\
    32, 31, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17, 16,\
    15, 14, 13, 12, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, 1, 0, CROBOT_ASSERT_NO_MACRO)
  #define CROBOT_ASSERT_NARG_( _0, _1, _2, _3, _4, _5, _6, _7, _8,\
                            _9, _10, _11, _12, _13, _14, _15, _16,\
                            _17, _18, _19, _20, _21, _22, _23, _24,\
                            _25, _26, _27, _28, _29, _30, _31, _32, _33, ...) _33

  #define CROBOT_ASSERT_HAS_ONE_ARG(...) CROBOT_ASSERT_APPLY_VA_ARGS(CROBOT_ASSERT_NARG_, CROBOT_ASSERT_NO_MACRO,##__VA_ARGS__,\
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,\
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, CROBOT_ASSERT_NO_MACRO)

  #if defined(__GNUC__) || defined(__clang__)
    #define CROBOT_ASSERT_LIKELY(arg) __builtin_expect(!!(arg), !0)
    #define CROBOT_ASSERT_UNLIKELY(arg) __builtin_expect(!!(arg), 0)
  #else
    #define CROBOT_ASSERT_LIKELY(arg) (arg)
    #define CROBOT_ASSERT_UNLIKELY(arg) (arg)
  #endif

  #define CROBOT_ASSERT_UNUSED(expression) (void)(true ? (void)0 : ((void)(expression)))

  #if !defined(CROBOT_ASSERT_DEBUG_BREAK)
    #if defined(_WIN32)
      extern void __cdecl __debugbreak(void);
      #define CROBOT_ASSERT_DEBUG_BREAK() __debugbreak()
    #else
      #if defined(__APPLE__)
        #include <TargetConditionals.h>
      #endif
      #if defined(__clang__) && !TARGET_OS_IPHONE || TARGET_IPHONE_SIMULATOR
        #define CROBOT_ASSERT_DEBUG_BREAK() __builtin_debugtrap()
      #elif defined(linux) || defined(__linux) || defined(__linux__) || defined(__APPLE__)
        #include <signal.h>
        #define CROBOT_ASSERT_DEBUG_BREAK() raise(SIGTRAP)
      #elif defined(__GNUC__)
        #define CROBOT_ASSERT_DEBUG_BREAK() __builtin_trap()
      #else
        #define CROBOT_ASSERT_DEBUG_BREAK() ((void)0)
      #endif
    #endif
  #endif

  #if (defined (__cplusplus) && (__cplusplus > 199711L)) || (defined(_MSC_FULL_VER) && (_MSC_FULL_VER >= 150020706))
    #define CROBOT_ASSERT_CXX11
  #endif

  #if defined(CROBOT_ASSERT_CXX11)
    #define CROBOT_ASSERT_NULLPTR nullptr
  #else
    #define CROBOT_ASSERT_NULLPTR 0
  #endif

  #define CROBOT_ASSERT_(level, ...)          CROBOT_ASSERT_JOIN(CROBOT_ASSERT_, CROBOT_ASSERT_HAS_ONE_ARG(__VA_ARGS__))(level, __VA_ARGS__)
  #define CROBOT_ASSERT_0(level, ...)         CROBOT_ASSERT_APPLY_VA_ARGS(CROBOT_ASSERT_2, level, __VA_ARGS__)
  #define CROBOT_ASSERT_1(level, expression)  CROBOT_ASSERT_2(level, expression, CROBOT_ASSERT_NULLPTR)

  #if defined(_MSC_FULL_VER) && (_MSC_FULL_VER >= 140050215)

    #if defined(CROBOT_ASSERT_DISABLE_IGNORE_LINE)

      #define CROBOT_ASSERT_3(level, expression, ...)\
        __pragma(warning(push))\
        __pragma(warning(disable: 4127))\
        do\
        {\
          if (CROBOT_ASSERT_LIKELY(expression) || crobot::common::implementation::ignoreAllAsserts());\
          else\
          {\
            if (crobot::common::implementation::handleAssert(CROBOT_ASSERT_FILE, CROBOT_ASSERT_LINE, CROBOT_ASSERT_FUNCTION, #expression, level, CROBOT_ASSERT_NULLPTR, __VA_ARGS__) == crobot::common::implementation::AssertAction::Break)\
              CROBOT_ASSERT_DEBUG_BREAK();\
          }\
        }\
        while (false)\
        __pragma(warning(pop))

    #else

      #define CROBOT_ASSERT_3(level, expression, ...)\
        __pragma(warning(push))\
        __pragma(warning(disable: 4127))\
        do\
        {\
          static bool _ignore = false;\
          if (CROBOT_ASSERT_LIKELY(expression) || _ignore || crobot::common::implementation::ignoreAllAsserts());\
          else\
          {\
            if (crobot::common::implementation::handleAssert(CROBOT_ASSERT_FILE, CROBOT_ASSERT_LINE, CROBOT_ASSERT_FUNCTION, #expression, level, &_ignore, __VA_ARGS__) == crobot::common::implementation::AssertAction::Break)\
              CROBOT_ASSERT_DEBUG_BREAK();\
          }\
        }\
        while (false)\
        __pragma(warning(pop))

    #endif

  #else

    #if (defined(__GNUC__) && ((__GNUC__ * 1000 + __GNUC_MINOR__ * 100) >= 4600)) || defined(__clang__)
      #define _pragma(x) _Pragma(#x)
      #define _CROBOT_ASSERT_WFORMAT_AS_ERROR_BEGIN\
        _pragma(GCC diagnostic push)\
        _pragma(GCC diagnostic error "-Wformat")
      #define _CROBOT_ASSERT_WFORMAT_AS_ERROR_END\
        _pragma(GCC diagnostic pop)
    #else
      #define _CROBOT_ASSERT_WFORMAT_AS_ERROR_BEGIN
      #define _CROBOT_ASSERT_WFORMAT_AS_ERROR_END
    #endif

    #if defined(CROBOT_ASSERT_DISABLE_IGNORE_LINE)

      #define CROBOT_ASSERT_3(level, expression, ...)\
        do\
        {\
          if (CROBOT_ASSERT_LIKELY(expression) || crobot::common::implementation::ignoreAllAsserts());\
          else\
          {\
            _CROBOT_ASSERT_WFORMAT_AS_ERROR_BEGIN\
            if (crobot::common::implementation::handleAssert(CROBOT_ASSERT_FILE, CROBOT_ASSERT_LINE, CROBOT_ASSERT_FUNCTION, #expression, level, CROBOT_ASSERT_NULLPTR, __VA_ARGS__) == crobot::common::implementation::AssertAction::Break)\
              CROBOT_ASSERT_DEBUG_BREAK();\
            _CROBOT_ASSERT_WFORMAT_AS_ERROR_END\
          }\
        }\
        while (false)

    #else

      #define CROBOT_ASSERT_3(level, expression, ...)\
        do\
        {\
          static bool _ignore = false;\
          if (CROBOT_ASSERT_LIKELY(expression) || _ignore || crobot::common::implementation::ignoreAllAsserts());\
          else\
          {\
            _CROBOT_ASSERT_WFORMAT_AS_ERROR_BEGIN\
            if (crobot::common::implementation::handleAssert(CROBOT_ASSERT_FILE, CROBOT_ASSERT_LINE, CROBOT_ASSERT_FUNCTION, #expression, level, &_ignore, __VA_ARGS__) == crobot::common::implementation::AssertAction::Break)\
              CROBOT_ASSERT_DEBUG_BREAK();\
            _CROBOT_ASSERT_WFORMAT_AS_ERROR_END\
          }\
        }\
        while (false)

    #endif

  #endif

  #define CROBOT_ASSERT_USED_(...)            CROBOT_ASSERT_USED_0(CROBOT_ASSERT_NARG(__VA_ARGS__), __VA_ARGS__)
  #define CROBOT_ASSERT_USED_0(N, ...)        CROBOT_ASSERT_JOIN(CROBOT_ASSERT_USED_, N)(__VA_ARGS__)

  #define CROBOT_STATIC_ASSERT(...)           CROBOT_ASSERT_APPLY_VA_ARGS(CROBOT_ASSERT_JOIN(CROBOT_STATIC_ASSERT_, CROBOT_ASSERT_HAS_ONE_ARG(__VA_ARGS__)), __VA_ARGS__)
  #if defined(CROBOT_ASSERT_CXX11)
    #define CROBOT_STATIC_ASSERT_0(expression, message) static_assert(expression, message)
  #else
    #define CROBOT_STATIC_ASSERT_0(expression, message)\
      struct CROBOT_ASSERT_JOIN(_crobot_static_assertion_at_line_, CROBOT_ASSERT_LINE)\
      {\
        crobot::common::implementation::StaticAssertion<static_cast<bool>((expression))> CROBOT_ASSERT_JOIN(STATIC_ASSERTION_FAILED_AT_LINE_, CROBOT_ASSERT_LINE);\
      };\
      typedef crobot::common::implementation::StaticAssertionTest<sizeof(CROBOT_ASSERT_JOIN(_crobot_static_assertion_at_line_, CROBOT_ASSERT_LINE))> CROBOT_ASSERT_JOIN(_crobot_static_assertion_test_at_line_, CROBOT_ASSERT_LINE)
      // note that we wrap the non existing type inside a struct to avoid warning
      // messages about unused variables when static assertions are used at function
      // scope
      // the use of sizeof makes sure the assertion error is not ignored by SFINAE
  #endif
  #define CROBOT_STATIC_ASSERT_1(expression)  CROBOT_STATIC_ASSERT_0(expression, #expression)

  #if !defined (CROBOT_ASSERT_CXX11)
    namespace crobot {
    namespace common {
    namespace implementation {

      template <bool>
      struct StaticAssertion;

      template <>
      struct StaticAssertion<true>
      {
      }; // StaticAssertion<true>

      template<int i>
      struct StaticAssertionTest
      {
      }; // StaticAssertionTest<int>

    } // namespace implementation
    } // namespace common
    } // namespace crobot
  #endif

  #if !defined(CROBOT_ASSERT_DISABLE_STL)
    #if defined(_MSC_VER)
      #pragma warning(push)
      #pragma warning(disable: 4548)
      #pragma warning(disable: 4710)
    #endif
    #include <stdexcept>
    #if defined(_MSC_VER)
      #pragma warning(pop)
    #endif
  #endif

  #if !defined(CROBOT_ASSERT_EXCEPTION_MESSAGE_BUFFER_SIZE)
    #define CROBOT_ASSERT_EXCEPTION_MESSAGE_BUFFER_SIZE 1024
  #endif

  #if defined(CROBOT_ASSERT_CXX11) && !defined(_MSC_VER)
    #define CROBOT_ASSERT_EXCEPTION_NO_THROW noexcept(true)
  #else
    #define CROBOT_ASSERT_EXCEPTION_NO_THROW throw()
  #endif

  #if defined(CROBOT_ASSERT_CXX11)
    #include <utility>
  #endif

  namespace crobot::common {

  #if !defined(CROBOT_ASSERT_DISABLE_STL)
    class AssertionException: public std::exception
  #else
    class AssertionException
  #endif
    {
      public:
      explicit AssertionException(const char* file,
                                  int line,
                                  const char* function,
                                  const char* expression,
                                  const char* message);

      AssertionException(const AssertionException& rhs);

      virtual ~AssertionException() CROBOT_ASSERT_EXCEPTION_NO_THROW;

      AssertionException& operator = (const AssertionException& rhs);

      virtual const char* what() const CROBOT_ASSERT_EXCEPTION_NO_THROW;

      const char* file() const;
      int line() const;
      const char* function() const;
      const char* expression() const;

      private:
      const char* _file;
      int _line;
      const char* _function;
      const char* _expression;

      enum
      {
        request = CROBOT_ASSERT_EXCEPTION_MESSAGE_BUFFER_SIZE,
        size = request > sizeof(char*) ? request : sizeof(char*) + 1
      };

      union
      {
        char  _stack[size];
        char* _heap;
      };

      CROBOT_STATIC_ASSERT(size > sizeof(char*), "invalid_size");
    }; // AssertionException

    CROBOT_ASSERT_ALWAYS_INLINE const char* AssertionException::file() const
    {
      return _file;
    }

    CROBOT_ASSERT_ALWAYS_INLINE int AssertionException::line() const
    {
      return _line;
    }

    CROBOT_ASSERT_ALWAYS_INLINE const char* AssertionException::function() const
    {
      return _function;
    }

    CROBOT_ASSERT_ALWAYS_INLINE const char* AssertionException::expression() const
    {
      return _expression;
    }

    namespace implementation {

    #if defined(_MSC_VER) && !defined(_CPPUNWIND)
      #if !defined(CROBOT_ASSERT_DISABLE_EXCEPTIONS)
        #define CROBOT_ASSERT_DISABLE_EXCEPTIONS
      #endif
    #endif

    #if !defined(CROBOT_ASSERT_DISABLE_EXCEPTIONS)

      template<typename E>
      inline void throwException(const E& e)
      {
        throw e;
      }

    #else

      // user defined, the behavior is undefined if the function returns
      void throwException(const crobot::common::AssertionException& e);

    #endif

    namespace AssertLevel {

      enum AssertLevel
      {
        Warning = 32,
        Debug   = 64,
        Error   = 128,
        Fatal   = 256

      }; // AssertLevel

    } // AssertLevel

    namespace AssertAction {

      enum AssertAction
      {
        None,
        Abort,
        Break,
        Ignore,
      #if !defined(CROBOT_ASSERT_DISABLE_IGNORE_LINE)
        IgnoreLine,
      #endif
        IgnoreAll,
        Throw

      }; // AssertAction

    } // AssertAction

    #if !defined(CROBOT_ASSERT_CALL)
      #define CROBOT_ASSERT_CALL
    #endif

    typedef AssertAction::AssertAction (CROBOT_ASSERT_CALL *AssertHandler)(const char* file,
                                                                        int line,
                                                                        const char* function,
                                                                        const char* expression,
                                                                        int level,
                                                                        const char* message);


  #if defined(__GNUC__) || defined(__clang__)
    #define CROBOT_ASSERT_HANDLE_ASSERT_FORMAT __attribute__((format (printf, 7, 8)))
  #else
    #define CROBOT_ASSERT_HANDLE_ASSERT_FORMAT
  #endif

  #if !defined(CROBOT_ASSERT_FUNCSPEC)
    #define CROBOT_ASSERT_FUNCSPEC
  #endif

    CROBOT_ASSERT_FUNCSPEC
    AssertAction::AssertAction CROBOT_ASSERT_CALL handleAssert(const char* file,
                                                            int line,
                                                            const char* function,
                                                            const char* expression,
                                                            int level,
                                                            bool* ignoreLine,
                                                            const char* message, ...) CROBOT_ASSERT_HANDLE_ASSERT_FORMAT;

    CROBOT_ASSERT_FUNCSPEC
    AssertHandler CROBOT_ASSERT_CALL setAssertHandler(AssertHandler handler);

    CROBOT_ASSERT_FUNCSPEC
    void CROBOT_ASSERT_CALL ignoreAllAsserts(bool value);

    CROBOT_ASSERT_FUNCSPEC
    bool CROBOT_ASSERT_CALL ignoreAllAsserts();

  #if defined(CROBOT_ASSERT_CXX11)

    template<int level, typename T>
    class AssertUsedWrapper
    {
      public:
      AssertUsedWrapper(T&& t);
      ~AssertUsedWrapper() CROBOT_ASSERT_EXCEPTION_NO_THROW;

      operator T();

      private:
      const AssertUsedWrapper& operator = (const AssertUsedWrapper&); // not implemented on purpose (and only VS2013 supports deleted functions)

      T t;
      mutable bool used;

    }; // AssertUsedWrapper<int, T>

    template<int level, typename T>
    inline AssertUsedWrapper<level, T>::AssertUsedWrapper(T&& _t)
      : t(std::forward<T>(_t)), used(false)
    {}

    template<int level, typename T>
    inline AssertUsedWrapper<level, T>::operator T()
    {
      used = true;
      return std::move(t);
    }

    template<int level, typename T>
    inline AssertUsedWrapper<level, T>::~AssertUsedWrapper() CROBOT_ASSERT_EXCEPTION_NO_THROW
    {
      CROBOT_ASSERT_3(level, used, "unused value");
    }

  #else

    template<int level, typename T>
    class AssertUsedWrapper
    {
      public:
      AssertUsedWrapper(const T& t);
      AssertUsedWrapper(const AssertUsedWrapper& rhs);
      ~AssertUsedWrapper() CROBOT_ASSERT_EXCEPTION_NO_THROW;

      operator T() const;

      private:
      const AssertUsedWrapper& operator = (const AssertUsedWrapper&); // not implemented on purpose

      T t;
      mutable bool used;

    }; // AssertUsedWrapper<int, T>

    template<int level, typename T>
    CROBOT_ASSERT_ALWAYS_INLINE AssertUsedWrapper<level, T>::AssertUsedWrapper(const T& _t)
      : t(_t), used(false)
    {}

    template<int level, typename T>
    CROBOT_ASSERT_ALWAYS_INLINE AssertUsedWrapper<level, T>::AssertUsedWrapper(const AssertUsedWrapper& rhs)
      : t(rhs.t), used(rhs.used)
    {}

    // /!\ GCC is not so happy if we inline that destructor
    template<int level, typename T>
    AssertUsedWrapper<level, T>::~AssertUsedWrapper() CROBOT_ASSERT_EXCEPTION_NO_THROW
    {
      CROBOT_ASSERT_3(level, used, "unused value");
    }

    template<int level, typename T>
    CROBOT_ASSERT_ALWAYS_INLINE AssertUsedWrapper<level, T>::operator T() const
    {
      used = true;
      return t;
    }

  #endif

  } // namespace implementation

  } // namespace crobot

#endif

#undef CROBOT_ASSERT_2
#undef CROBOT_ASSERT_USED_1
#undef CROBOT_ASSERT_USED_2

#if defined(_MSC_VER) && defined(_PREFAST_)

  #define CROBOT_ASSERT_2(level, expression, ...) __analysis_assume(!!(expression))
  #define CROBOT_ASSERT_USED_1(type)              type
  #define CROBOT_ASSERT_USED_2(level, type)       type

#elif defined(__clang__) && defined(__clang_analyzer__)

  void its_going_to_be_ok(bool expression) __attribute__((analyzer_noreturn));
  #define CROBOT_ASSERT_2(level, expression, ...) its_going_to_be_ok(!!(expression))
  #define CROBOT_ASSERT_USED_1(type)              type
  #define CROBOT_ASSERT_USED_2(level, type)       type

#else

  #if CROBOT_ASSERT_ENABLED

    #define CROBOT_ASSERT_2(level, expression, ...) CROBOT_ASSERT_3(level, expression, __VA_ARGS__)
    #define CROBOT_ASSERT_USED_1(type)              crobot::common::implementation::AssertUsedWrapper<crobot::common::implementation::AssertLevel::CROBOT_ASSERT_DEFAULT_LEVEL, type>
    #define CROBOT_ASSERT_USED_2(level, type)       crobot::common::implementation::AssertUsedWrapper<level, type>

  #else

    #define CROBOT_ASSERT_2(level, expression, ...) CROBOT_ASSERT_UNUSED(expression)
    #define CROBOT_ASSERT_USED_1(type)              type
    #define CROBOT_ASSERT_USED_2(level, type)       type

  #endif

#endif

#if (defined(__GNUC__) && ((__GNUC__ * 1000 + __GNUC_MINOR__ * 100) >= 4600)) || defined(__clang__)
  #pragma GCC diagnostic pop
#endif
// clang-format on
