# 🌿 cobot3_ws Git 협업 컨벤션

브랜치, 커밋, 푸시, PR은 모두 이 규칙을 따릅니다.

---

## 1. 브랜치 이름

**형식:** `<type>/<이슈번호>-<설명>`

```bash
feature/123-login-api
fix/245-order-null-check
hotfix/301-payment-timeout
refactor/88-user-service
chore/12-gradle-upgrade
```

- 소문자와 하이픈만 씁니다. 공백, 대문자, 한글은 쓰지 않습니다.
- 이슈 번호를 넣어 이슈와 연결합니다. 이슈가 없으면 `<type>/<설명>`으로 씁니다.
- 브랜치 하나에는 작업 하나만 담습니다. 머지되면 바로 삭제합니다.
- main에 직접 push하지 않습니다.

---

## 2. 커밋 메시지

```text
type(scope): 요약 (50자 이내)

변경 이유를 설명하는 본문
Closes #245
```

| type | 용도 |
| --- | --- |
| feat | 새 기능 |
| fix | 버그 수정 |
| refactor | 동작 변화가 없는 구조 개선 |
| test | 테스트 추가 또는 수정 |
| docs | 문서 |
| chore | 빌드, 설정, 의존성 |

- 커밋은 작게, 의미 단위로 나눕니다.
- ❌ "수정", "asdf", "최종_진짜최종" 같은 메시지는 쓰지 않습니다.

---

## 3. 하루 작업 흐름

1. **동기화**: `git switch main && git pull`로 항상 최신 main에서 시작합니다.
2. **브랜치**: `git switch -c feature/123-login`으로 작업마다 새 브랜치를 만듭니다.
3. **커밋**: `git add -p && git commit`으로 작게, 자주 커밋합니다.
4. **푸시**: `git push -u origin HEAD`로 원격에 올립니다. 백업 겸 공유입니다.
5. **PR**: `gh pr create`로 PR을 만들고 템플릿을 채운 뒤 리뷰어를 지정합니다.
6. **리뷰 반영**: 리뷰에서 받은 수정은 추가 커밋으로 올립니다.
7. **머지**: Squash and merge로 머지한 뒤 브랜치를 삭제합니다.

---

## 4. PR

- 리뷰어가 한 번에 읽을 수 있는 크기로 만듭니다. 크면 나눠서 올립니다.
- 올리기 전에 diff를 직접 훑어보고 디버그 코드를 지웁니다.

**PR 템플릿**

```markdown
## 작업 내용
## 변경 이유
## 테스트 방법
## 리뷰 포인트
Closes #N
```

---

## 5. 통합과 충돌

- feature 브랜치를 main에 넣을 때는 **squash merge**를 씁니다.
- 공유 브랜치에서는 rebase하지 않습니다.
- `--force`는 쓰지 않습니다. 강제로 push해야 하면 `--force-with-lease`를 씁니다.

```bash
# 1. main 최신 내용 위로 올리기
git fetch origin
git rebase origin/main

# 2. 충돌 마커(<<<<<<< ======= >>>>>>>)를 정리한 뒤 계속하기
git add <file>
git rebase --continue

# 3. 테스트가 통과하면 안전하게 푸시하기
git push --force-with-lease
```

---

## 6. 되돌리기

| 상황 | 명령 | 메모 |
| --- | --- | --- |
| add 전 변경 버리기 | `git restore <file>` | 변경 내용이 사라짐 |
| add 취소 | `git restore --staged <file>` | 변경은 유지됨 |
| 마지막 커밋 수정 | `git commit --amend` | push 전에만 |
| 로컬 커밋 취소 | `git reset --soft HEAD~1` | 변경은 staged로 남음 |
| 이미 push한 커밋 취소 | `git revert <commit>` | **공유 브랜치에서는 이것만 씀** |
| 날아간 커밋 찾기 | `git reflog` | HEAD 이동 기록에서 복구 |

---

## ✅ 푸시 전 체크리스트

- [ ] 브랜치 이름이 `<type>/<이슈번호>-<설명>` 형식인가?
- [ ] 커밋 메시지가 `type(scope): 요약` 형식인가?
- [ ] 최신 main을 반영했는가?
- [ ] diff를 셀프 리뷰하고 디버그 코드를 지웠는가?
- [ ] PR 템플릿을 채웠는가?
