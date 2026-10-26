"""Native Python editor: prompt -> review -> validate -> preview/test -> save."""
import json,time
from pathlib import Path

def proposal_text(controller):
    if controller.proposal is not None:
        p=controller.proposal;view=controller.presentation();lines=[view['status'],p['title'],'',p['summary']]
        if view['snapshot']:
            snapshot=view['snapshot']
            lines.extend(['','表示対象の制御設定:',json.dumps(dict(tuning=snapshot['tuning'],controller=snapshot.get('controller')),ensure_ascii=False,indent=2)])
        if p.get('diagnostic_analysis'):lines.extend(['','段階的な原因分析・参考資料:',json.dumps(p['diagnostic_analysis'],ensure_ascii=False,indent=2)])
        if p.get('constraint_review'):lines.extend(['','元の作成条件との照合:',json.dumps(p['constraint_review'],ensure_ascii=False,indent=2)])
        if 'controller_code' in p:
            lines.extend(['','機体MJCF:',p['mjcf'],'','制御コード:',p['controller_code'],'','駆動仕様:',json.dumps(p['joints'],ensure_ascii=False,indent=2)])
            if controller.last_result:lines.extend(['','診断結果:',json.dumps(controller.last_result,ensure_ascii=False,indent=2)])
            return '\n'.join(lines)
        result=controller.last_result
        if result:
            lines.extend(['','現在モデルの診断結果:'])
            for candidate in result.get('results',[]):
                lines.append('合格' if candidate.get('passed') else '失敗／未完了')
                for trial in candidate.get('trials',[]):
                    lines.append(f"・{trial['case']}: {'合格' if trial['passed'] else '失敗'} / {trial['simulation_seconds']:.2f}秒")
            if result.get('error'):lines.append(str(result['error']))
        lines.extend(['','機体の変更:'])
        names={'pos':'位置 [m]','size':'寸法 [m]','mass':'質量 [kg]','friction':'摩擦','rgba':'色','fromto':'端点 [m]','range':'可動域 [度]','damping':'減衰','armature':'付加慣性'}
        lines.extend(f"・{c['name']} / {names[c['attribute']]} → {c['value']}" for c in p['model_changes'])
        if not p['model_changes']:lines.append('・形状の変更なし')
        lines.extend(['','制御設定の変更:'])
        lines.extend(f"・駆動 {c['key']} → {c['value']}" for c in p['drive_changes'])
        lines.extend(f"・{c['profile']} / {c['key']} → {c['value']}" for c in p['motion_changes'])
        return '\n'.join(lines)
    if controller.current:
        return '現在のモデル・制御設定\n'+json.dumps(dict(source=controller.current['source_robot'],parent=controller.current.get('revision_id'),tuning=controller.current['tuning'],controller=controller.current.get('controller'),diagnostic=controller.last_result,creation_prompt=controller.current.get('creation_prompt'),diagnostic_analysis=controller.current.get('proposal',{}).get('diagnostic_analysis')),ensure_ascii=False,indent=2)
    return '変更案がここに表示されます。'

def show_workshop(controller, screenshot=None, smoke=False, metrics_path=None):
    from xrobocon.workshop.qt_window import run_workshop
    return run_workshop(controller, screenshot, smoke, metrics_path)
