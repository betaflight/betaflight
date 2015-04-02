<?
	// see docs/Travis.md, .travis.sh

	$baseDir = "/var/www/builds/";

	$myFile = $_FILES["file"];
	$recentCommits = $_POST["recent_commits"];
	$travisJobId = sanitize($_POST["travis_build_number"]);
	$lastCommitDate = sanitize($_POST["last_commit_date"]);
	$revision = sanitize($_POST["revision"]);
	$branch = sanitize($_POST["branch"]);

	$uploadDir = $baseDir . "/" . $lastCommitDate . "/";
	$prefix = $uploadDir . $travisJobId . "_" . $revision;

	if(!file_exists($uploadDir)) mkdir($uploadDir, 0660, true);

	if($myFile) {
		$uploadfile = $prefix . "_" . (basename($myFile['name']));
		if(move_uploaded_file($myFile['tmp_name'], $uploadfile)) {
			echo "upload succeeded.\n";
		}
		else {
			echo "upload failed $uploadfile\n";
		}
	}

	if($revision && $lastCommitDate && $recentCommits) {
		$changelog = fopen($prefix . "_changes.txt", "w") or die ("unable to open changelog file for writing");
		fwrite($changelog, $recentCommits);
		fclose($changelog);
	}

	print_r($_FILES);
	print_r($_POST);
	print_r($_GET);

	function sanitize($str) {
		return (preg_replace('/[^A-Za-z0-9_\-]/', '_', ($str)));
	}

?>
