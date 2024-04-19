/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * GUO Zhichong <guozhichong@sensetime.com>
 */
#ifndef WITH_TDA_QNX
#include <elf.h>       // Elf64_Ehdr/Elf64_Shdr
#include <execinfo.h>  // backtrace
#else
#include <sys/elf.h>
#include <backtrace.h>
#endif
#include <signal.h>    // signal
#include <unistd.h>    // sleep/usleep
#include <dlfcn.h>     // dladdr
#include <cxxabi.h>    // abi::__cxa_demangle
#include <pthread.h>   // pthread_t
#include <fcntl.h>     // open
#include <sys/stat.h>  // O_RDONLY

#include <atomic>

#include "ad_log/ad_log.hpp"
#include "ad_common/data_type/base.hpp"
#include "ad_utils/signal_handler.hpp"

namespace senseAD {
namespace ad_utils {

namespace {

static std::atomic_flag kInstallFailureSignalHandler{ATOMIC_FLAG_INIT};

std::atomic<pthread_t *> old_pthread{nullptr};

const struct {
    int32_t signum;
    const char *signame;
} kSignalTable[] = {
    {SIGHUP, "SIGHUP"},   {SIGSEGV, "SIGSEGV"}, {SIGILL, "SIGILL"},
    {SIGABRT, "SIGABRT"}, {SIGFPE, "SIGFPE"},
};

static constexpr std::size_t kNumSignalTable =
    sizeof(kSignalTable) / sizeof(*kSignalTable);
}  // namespace

void StripNameInPlace(char *input_output, std::size_t length) {
    static constexpr std::size_t kBufferSize = 512;
    char buffer[kBufferSize];  // large enough
    memset(buffer, 0, kBufferSize * sizeof(char));
    char *head = buffer;
    char *tail = buffer + kBufferSize;
    char *cursor = input_output;
    char *input_end = input_output + length;
    int32_t left_bracket = 0;
    int32_t left_angle_bracket = 0;
    while (*cursor != '\0' && cursor != input_end && head != tail) {
        if ('<' == *cursor) {
            ++left_angle_bracket;
            if (left_angle_bracket <= 1 && left_bracket < 1) {
                *(head++) = *cursor;
            }
        } else if ('>' == *cursor) {
            if (left_angle_bracket <= 1 && left_bracket < 1) {
                *(head++) = *cursor;
            }
            --left_angle_bracket;
        } else if ('(' == *cursor) {
            ++left_bracket;
            if (left_bracket <= 1) {
                *(head++) = *cursor;
            }
        } else if (')' == *cursor) {
            if (left_bracket <= 1) {
                *(head++) = *cursor;
            }
            --left_bracket;
        } else {
            if (left_bracket <= 0 && left_angle_bracket <= 0) {
                *(head++) = *cursor;
            }
        }
        cursor++;
    }
    strncpy(input_output, buffer, length < kBufferSize ? length : kBufferSize);
}

bool GetSectionHeaderByType(int32_t fd,
                            const Elf64_Ehdr &elf_header,
                            Elf64_Shdr *section_header,
                            uint32_t sh_type) {
    if (nullptr == section_header) {
        return false;
    }
    uint64_t sections_start_offset = elf_header.e_shoff;
    uint16_t num_of_sections = elf_header.e_shnum;
    uint16_t size_of_section_header = elf_header.e_shentsize;
    // TODO(someone): reduce 'read', could try batch read.
    for (std::size_t i = 0; i < num_of_sections; ++i) {
        Elf64_Shdr header;
        auto bytes = pread(fd, &header, sizeof(header),
                           sections_start_offset + i * size_of_section_header);
        if (sizeof(header) != bytes) {
            return false;
        }
        if (sh_type != header.sh_type) {
            continue;
        }
        *section_header = header;
        return true;
    }
    return false;
}

bool Demangle(int32_t object_fd,
              const Elf64_Ehdr &elf_header,
              const Elf64_Shdr &symtab,
              const Elf64_Shdr &strtab,
              void *ip,
              char *output,
              std::size_t output_size,
              uint32_t offset) {
    std::size_t num_of_symbols = symtab.sh_size / symtab.sh_entsize;
    // TODO(someone): reduce 'read'
    for (std::size_t i = 0; i < num_of_symbols; ++i) {
        Elf64_Sym symbol_entry;
        auto read_bytes = pread(object_fd, &symbol_entry, sizeof(symbol_entry),
                                symtab.sh_offset + i * symtab.sh_entsize);
        if (sizeof(symbol_entry) != read_bytes) {
            return false;
        }
        uint32_t start_address = symbol_entry.st_value;
        start_address += offset;
        uint32_t end_address = start_address + symbol_entry.st_size;
        uint32_t pc = reinterpret_cast<std::uintptr_t>(ip);
        if (symbol_entry.st_value != 0 && symbol_entry.st_shndx != 0 &&
            start_address <= pc && end_address >= pc) {
            char symbol[256];  // large enough
            auto symbol_length = pread(object_fd, symbol, sizeof(symbol),
                                       strtab.sh_offset + symbol_entry.st_name);
            strncpy(output, symbol,
                    output_size > static_cast<std::size_t>(symbol_length)
                        ? symbol_length
                        : output_size);
            int32_t status = -1;
            const char *demangle_name =
                abi::__cxa_demangle(symbol, NULL, NULL, &status);
            if (0 == status) {
                std::size_t len = 0;
                while ('\0' != *(demangle_name + len)) {
                    ++len;
                }
                auto output_length =
                    len + 1 > output_size ? output_size : len + 1;
                strncpy(output, demangle_name, output_length);
                // Output the full de-mangled name
                // StripNameInPlace(output, output_length);
            }
            return true;
        }
    }
    return false;
}

bool SymbolizeAndDemangle(void *ip,
                          char *output,
                          std::size_t output_size,
                          char *filepath = nullptr) {
    const char unknown[] = "(unknown)";
    strncpy(output, unknown,
            output_size > sizeof(unknown) ? sizeof(unknown) : output_size);
    Dl_info info;
    // succeed if != 0 else failed
    if (0 == dladdr(ip, &info)) {
        return false;
    }
    if (nullptr != filepath) {
        const char *cursor = info.dli_fname;
        while ('\0' != *cursor) {
            cursor++;
        }
        strncpy(filepath, info.dli_fname, cursor + 1 - info.dli_fname);
    }

    int32_t object_fd = open(info.dli_fname, O_RDONLY);
    if (object_fd < 0) {
        return false;
    }
    Elf64_Ehdr elf_header;
    auto bytes = pread(object_fd, &elf_header, sizeof(elf_header), 0);
    if (sizeof(elf_header) != bytes) {
        return false;
    }

    uint32_t offset = 0;
    if (elf_header.e_type == ET_DYN) {
        // If it's a shared object, set base address to loaded address
        offset = reinterpret_cast<std::uintptr_t>(info.dli_fbase);
    }

    Elf64_Shdr symtab, strtab;
    if (GetSectionHeaderByType(object_fd, elf_header, &symtab, SHT_SYMTAB)) {
        auto ret =
            pread(object_fd, &strtab, sizeof(strtab),
                  elf_header.e_shoff + symtab.sh_link * elf_header.e_shentsize);
        if (sizeof(strtab) != ret) {
            AD_LERROR(STACKTRACE)
                << "Failed to read string tab of " << info.dli_fname;
            return false;
        } else {
            if (Demangle(object_fd, elf_header, symtab, strtab, ip, output,
                         output_size, offset)) {
                return true;
            }
        }
    }

    if (GetSectionHeaderByType(object_fd, elf_header, &symtab, SHT_DYNSYM)) {
        auto ret =
            pread(object_fd, &strtab, sizeof(strtab),
                  elf_header.e_shoff + symtab.sh_link * elf_header.e_shentsize);
        if (sizeof(strtab) != ret) {
            AD_LERROR(STACKTRACE)
                << "Failed to read string tab of " << info.dli_fname;
            return false;
        } else {
            if (!Demangle(object_fd, elf_header, symtab, strtab, ip, output,
                          output_size, offset)) {
                return false;
            }
        }
    }
    return false;
}

int32_t StackTrace(void **stack, int32_t max_depth) {
    static constexpr int32_t kMaxStackDepth = 64;
#ifndef WITH_TDA_QNX
    static void *buffer[kMaxStackDepth];
    auto depth = backtrace(buffer, kMaxStackDepth);

    max_depth = max_depth > depth - 1 ? depth - 1 : max_depth;
    for (int32_t i = 0; i < max_depth; ++i) {
        stack[i] = buffer[i + 1];
    }
#else
    bt_accessor_t acc;
    bt_init_accessor(&acc, BT_SELF);
    bt_addr_t pc[kMaxStackDepth];

    auto depth = bt_get_backtrace(&acc, pc, kMaxStackDepth);
    max_depth = max_depth > depth - 1 ? depth - 1 : max_depth;

    for (int32_t i = 0; i < max_depth; ++i) {
        stack[i] = reinterpret_cast<void *>(pc[i]);
    }
#endif

    return max_depth;
}

void InvokeDefaultHandler(int32_t signum) {
    struct sigaction sig_action;
    sig_action.sa_handler = SIG_DFL;
    memset(&sig_action, 0, sizeof(sig_action));
    sigemptyset(&sig_action.sa_mask);
    sigaction(signum, &sig_action, NULL);
    kill(getpid(), signum);
}

void FailureHandler(int32_t signum) {
    const char *signame = "unknown";
    for (std::size_t i = 0; i < kNumSignalTable; ++i) {
        if (signum == kSignalTable[i].signum) {
            signame = kSignalTable[i].signame;
        }
    }

    auto this_thread = pthread_self();
    pthread_t *old_val{nullptr};
    if (!old_pthread.compare_exchange_strong(old_val, &this_thread)) {
        // pending...
        if (old_val == &this_thread) {
            InvokeDefaultHandler(signum);
        }
        while (true) {
            sleep(1);
        }
    }
    // No handle on sighup
    if (signum != SIGHUP) {
        static constexpr int32_t kMaxStackTraceDepth = 32;
        void *stack[kMaxStackTraceDepth];
        auto depth = StackTrace(stack, kMaxStackTraceDepth);
        AD_LERROR(STACKTRACE) << " -------------------------------- ";
        AD_LERROR(STACKTRACE) << "* Stacktrace: " << signame;
        AD_LERROR(STACKTRACE) << "  Depth: " << depth;
        for (int32_t i = 1; i < depth; ++i) {
            char filepath[256];  // large enough to save filepath
            char out[1024];      // large enough for a insane de-mangled name
            memset(out, 0, sizeof(out));
            SymbolizeAndDemangle(stack[i + 1], out, sizeof(out), filepath);
            AD_LERROR(STACKTRACE)
                << "FROM: 0x" << std::hex
                << reinterpret_cast<std::uintptr_t>(stack[i + 1]) << " " << out
                << " FILE: " << filepath;
        }
    }
    extern void Shutdown() __attribute__((weak));
    if (Shutdown) {
        AD_LERROR(STACKTRACE) << "Call shutdown...";
        Shutdown();
    }
    LogManager::Instance().Flush();
    FileLogManager::Instance().Flush();
    InvokeDefaultHandler(signum);
}

void InstallFailureSignalHandler() {
    if (kInstallFailureSignalHandler.test_and_set()) {
        return;
    }
    struct sigaction sig_action;
    memset(&sig_action, 0, sizeof(sig_action));
    sigemptyset(&sig_action.sa_mask);
    sig_action.sa_handler = &FailureHandler;
    for (std::size_t i = 0; i < kNumSignalTable; ++i) {
        if (0 != sigaction(kSignalTable[i].signum, &sig_action, NULL)) {
            AD_LERROR(SIGNAL_HANDLER) << "Failed to set sigaction for signal "
                                      << kSignalTable[i].signame;
        }
    }
    return;
}

}  // namespace ad_utils
}  // namespace senseAD
