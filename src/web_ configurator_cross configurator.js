import PresetsRepoIndexed from "./PresetsRepoIndexed";

export default class PresetsWebsiteRepo extends PresetsRepoIndexed {
    constructor(url, official, name) {
        let correctUrl = url.trim();

        if (!correctUrl.endsWith("/")) {
            correctUrl += "/";
        }

        const urlRaw = correctUrl;
        const urlViewOnline = correctUrl;

        super(urlRaw, urlViewOnline, official, name);
    }
}