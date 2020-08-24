checks: check-target-independence \
	check-fastdata-usage-correctness \
	check-platform-included \
	check-unified-target-naming

check-target-independence:
	$(V1) for test_target in $(VALID_TARGETS); do \
		FOUND=$$(grep -rE "\W$${test_target}(\W.*)?$$" src/main | grep -vE "(//)|(/\*).*\W$${test_target}(\W.*)?$$" | grep -vE "^src/main/target"); \
		if [ "$${FOUND}" != "" ]; then \
			echo "Target dependencies for target '$${test_target}' found:"; \
			echo "$${FOUND}"; \
			exit 1; \
		fi; \
	done

check-fastdata-usage-correctness:
	$(V1) NON_TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_DATA_ZERO_INIT\W.*=.*" src/main/ | grep -Ev "=\s*(false|NULL|0(\.0*f?)?)\s*[,;]"); \
	if [ "$${NON_TRIVIALLY_INITIALIZED}" != "" ]; then \
		echo "Non-trivially initialized FAST_DATA_ZERO_INIT variables found, use FAST_DATA instead:"; \
		echo "$${NON_TRIVIALLY_INITIALIZED}"; \
		exit 1; \
	fi; \
	TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_DATA\W.*;" src/main/ | grep -v "="); \
	EXPLICITLY_TRIVIALLY_INITIALIZED=$$(grep -Ern "\W?FAST_DATA\W.*;" src/main/ | grep -E "=\s*(false|NULL|0(\.0*f?)?)\s*[,;]"); \
	if [ "$${TRIVIALLY_INITIALIZED}$${EXPLICITLY_TRIVIALLY_INITIALIZED}" != "" ]; then \
		echo "Trivially initialized FAST_DATA variables found, use FAST_DATA_ZERO_INIT instead to save FLASH:"; \
		echo "$${TRIVIALLY_INITIALIZED}\n$${EXPLICITLY_TRIVIALLY_INITIALIZED}"; \
		exit 1; \
	fi

check-platform-included:
	$(V1) PLATFORM_NOT_INCLUDED=$$(find src/main -type d -name target -prune -o -type f -name \*.c -exec grep -L "^#include \"platform.h\"" {} \;); \
	if [ "$$(echo $${PLATFORM_NOT_INCLUDED} | grep -v -e '^$$' | wc -l)" -ne 0 ]; then \
		echo "The following compilation units do not include the required target specific configuration provided by 'platform.h':"; \
		echo "$${PLATFORM_NOT_INCLUDED}"; \
		exit 1; \
	fi

check-unified-target-naming:
	$(V1) for target_config in unified_targets/configs/*; do \
		if [ -f $${target_config} ] && [ $$(sed -n 's/board_name \([A-Z]*\)/\1/p' $${target_config}).config != $$(basename $${target_config}) ]; then \
			echo "Invalid board name ($$(sed -n 's/board_name \([A-Z]*\)/\1/p' $${target_config})) in Unified Target configuration $${target_config}."; \
			exit 1; \
		fi; \
	done
