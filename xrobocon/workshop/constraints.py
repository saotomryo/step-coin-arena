"""Creation-intent provenance and validation of AI's explicit constraint assessment."""
import json
from pathlib import Path
from xrobocon.workshop.design import digest


def restore_creation_prompt(manifest, revisions):
    if 'creation_prompt' in manifest:return
    ancestor=manifest;visited=set();root=Path(revisions).resolve()
    while ancestor.get('parent') and ancestor['parent'] not in visited:
        parent=ancestor['parent'];visited.add(parent)
        path=(root/str(parent)/'manifest.json').resolve()
        if not path.is_relative_to(root) or not path.is_file():break
        ancestor=json.loads(path.read_text())
        if 'creation_prompt' in ancestor:
            manifest['creation_prompt']=ancestor['creation_prompt']
            manifest['creation_prompt_source']=ancestor.get('creation_prompt_source','ancestor_manifest')
            return
    prompt=ancestor.get('prompt')
    if ancestor.get('mode')=='new_robot' and isinstance(prompt,str) and prompt.strip():
        manifest['creation_prompt']=prompt;manifest['creation_prompt_source']='legacy_creation_revision:'+str(ancestor.get('id'))
    else:
        manifest['creation_prompt']=None;manifest['creation_prompt_source']='unavailable'


def validate_review(review,prompt):
    if not isinstance(review,dict) or set(review)!={'prompt_sha256','complies','summary','checks'}:raise ValueError('元の作成条件の照合結果が必要です')
    if review['prompt_sha256']!=digest((prompt or '').encode()):raise ValueError('制約確認が現在の作成プロンプトと一致しません')
    if review['complies'] is not True:raise ValueError('元の作成条件に適合しない変更案は適用しません')
    if not isinstance(review['summary'],str) or not 1<=len(review['summary'])<=3000:raise ValueError('制約確認の説明が必要です')
    checks=review['checks']
    if not isinstance(checks,list) or len(checks)>40 or prompt and not checks:raise ValueError('元の作成条件との照合項目が必要です')
    if not prompt and checks:raise ValueError('作成プロンプト不明の機体に元の制約を推測で追加できません')
    for check in checks:
        if not isinstance(check,dict) or set(check)!={'requirement','verdict','requires_simulation','detail'}:raise ValueError('invalid constraint check')
        for key in ('requirement','detail'):
            if not isinstance(check[key],str) or not 1<=len(check[key])<=2000:raise ValueError('invalid constraint explanation')
        if type(check['requires_simulation']) is not bool:raise ValueError('invalid simulation flag')
        if check['verdict'] not in ('preserved','violated','needs_test'):raise ValueError('invalid constraint verdict')
        if check['verdict']=='violated' or check['verdict']=='needs_test' and not check['requires_simulation']:
            raise ValueError('元の作成条件に違反、または構造条件が未確認です: '+check['requirement'])
    return review
