checks: check-target-independence \
	check-fastdata-usage-correctness \
	check-platform-included \
	check-stale-submodule-paths

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

# Catches a common branch-switch trap: a directory was promoted to a submodule on
# one branch, but the working tree still has the old embedded files from an earlier
# branch (or vice versa). The build then fails with cryptic header/source errors.
# We flag any path listed in .gitmodules that exists as a non-empty directory but
# is missing the submodule .git marker — i.e. it has content that doesn't belong
# to a hydrated submodule. Tells the user to clean the path and re-hydrate.
check-stale-submodule-paths:
	$(V1) SENTINEL=$$(mktemp); $(RM) $$SENTINEL; \
	git config --file .gitmodules --get-regexp '\.path$$' 2>/dev/null | awk '{print $$2}' | \
	while IFS= read -r p; do \
		[ -d "$$p" ] || continue; \
		[ -e "$$p/.git" ] && continue; \
		[ -z "$$(ls -A "$$p" 2>/dev/null)" ] && continue; \
		echo "STALE: $$p contains files but is not a hydrated submodule on this branch"; \
		echo "  fix: rm -rf $$p && git submodule update --init --checkout --recursive -- $$p"; \
		echo "stale" > $$SENTINEL; \
	done; \
	if [ -f $$SENTINEL ]; then \
		$(RM) $$SENTINEL; \
		exit 1; \
	fi
